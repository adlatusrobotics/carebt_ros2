# Copyright 2021 Andreas Steck (steck.andi@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import threading
from datetime import datetime
from typing import TYPE_CHECKING

from action_msgs.msg import GoalStatus
from carebt.actionNode import ActionNode
from carebt.nodeStatus import NodeStatus
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, NumberOfEntities
from rclpy._rclpy_pybind11 import InvalidHandle, RCLError
from rclpy.task import Future

_logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from carebt.behaviorTreeRunner import BehaviorTreeRunner  # pragma: no cover


class _SafeActionClient(ActionClient):
    """ActionClient subclass that guards against race conditions with the executor.

    Addresses two race conditions between the BT thread and the executor thread:

    1. **Creation race** – rclpy's ``ActionClient.__init__`` registers the
       client as a waitable (visible to the executor) *before* assigning
       ``self._lock``.  Pre-initializing ``_lock`` closes this window.

    2. **Destruction race** – when the BT thread calls ``destroy()`` the
       underlying handle is invalidated, but the executor may still hold a
       reference in its snapshot of waitables and call ``get_num_entities``
       or ``add_to_wait_set`` on the destroyed client.  Overriding both
       methods to catch the resulting exceptions prevents the crash.
    """

    def __init__(self, *args, **kwargs):
        self._lock = threading.Lock()
        super().__init__(*args, **kwargs)

    def get_num_entities(self) -> NumberOfEntities:
        """Return number of entities, returning zeros if the handle was destroyed."""
        try:
            return super().get_num_entities()
        except (InvalidHandle, RCLError) as e:
            _logger.debug('_SafeActionClient.get_num_entities skipped (destroyed): %s', e)
            return NumberOfEntities(0, 0, 0, 0, 0, 0)

    def add_to_wait_set(self, wait_set) -> None:
        """Add entities to wait set, silently skipping if the handle was destroyed."""
        try:
            super().add_to_wait_set(wait_set)
        except (InvalidHandle, RCLError) as e:
            _logger.debug('_SafeActionClient.add_to_wait_set skipped (destroyed): %s', e)


class RosActionClientActionNode(ActionNode):

    def __init__(self,
                 bt_runner: 'BehaviorTreeRunner',
                 action_type,
                 action_name: str,
                 params: str = None):
        super().__init__(bt_runner, params)
        self.set_status(NodeStatus.IDLE)
        self._goal_handle: ClientGoalHandle = None
        self._goal_msg = action_type.Goal()
        self._action_client = _SafeActionClient(bt_runner.node, action_type, action_name)
        self.get_logger().debug('{} - action_client.wait_for_server...'
                                .format(self.__class__.__name__))
        self._action_client.wait_for_server()  # TODO: Timeout
        self.get_logger().debug('{} - action_client.wait_for_server -> OK'
                                .format(self.__class__.__name__))

    # PRIVATE

    def __goal_response_callback(self, future: Future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn(f'{self.__class__.__name__} - Goal rejected')
            return

        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._internal_result_callback)

    # PROTECTED

    def _internal_on_abort(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal()
        super()._internal_on_abort()

    def _internal_on_delete(self) -> None:
        if hasattr(self, '_get_result_future') and self._get_result_future is not None:
            self._get_result_future._callbacks = []
        self._action_client._feedback_callbacks = {}
        # Do NOT call self._action_client.destroy() here.
        # destroy() invalidates the underlying C handle immediately, but the
        # executor thread may still hold a reference to this waitable in its
        # snapshot and crash when it tries to poll it.  Instead, just
        # deregister the client from the node so the executor won't pick it up
        # in future spin cycles.  The C handle stays valid; Python GC will
        # clean it up once all references are gone.
        self.bt_runner.node.remove_waitable(self._action_client)
        super()._internal_on_delete()

    def _internal_result_callback(self, future: Future) -> None:
        if future.cancelled():
            self.get_logger().debug(f'{self.__class__.__name__} - goal cancelled')
        else:
            status = future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().debug(f'{self.__class__.__name__} - goal succeeded')
                self.result_callback(future)
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().debug(f'{self.__class__.__name__} - goal aborted')
                # we can not distinguish between an abort due to a new goal (old goal aborted)
                # and a real abort of a goal due to a failure in the processing on the server side
                self.abort_callback(future)

    def _internal_on_tick(self) -> None:
        current_ts = datetime.now()
        if(self._throttle_ms is None or
                int((current_ts - self._last_ts).total_seconds() * 1000) >= self._throttle_ms):
            if(self.get_status() == NodeStatus.IDLE or
                    self.get_status() == NodeStatus.RUNNING):
                self.bt_runner.get_logger().trace('ticking {} - {}'
                                                  .format(self.__class__.__name__,
                                                          self.get_status()))
                if self.get_status() == NodeStatus.IDLE:
                    self.set_status(NodeStatus.RUNNING)
                self.on_tick()
                self._last_ts = current_ts

                if(self._goal_msg is not None):
                    self._goal_future = self._action_client\
                                            .send_goal_async(self._goal_msg,
                                                            feedback_callback=self.feedback_callback)
                    self._goal_future.add_done_callback(self.__goal_response_callback)

    # PUBLIC

    def result_callback(self, future) -> None:
        pass

    def abort_callback(self, future) -> None:
        pass

    def feedback_callback(self, msg) -> None:
        pass
