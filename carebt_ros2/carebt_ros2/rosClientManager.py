# Copyright 2026 Andreas Steck (steck.andi@gmail.com)
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
from typing import Any, Callable
from uuid import uuid4

from rclpy.action import ActionClient
from rclpy.action.client import NumberOfEntities
from rclpy.client import Client
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription
from rclpy._rclpy_pybind11 import InvalidHandle, RCLError

_logger = logging.getLogger(__name__)


class _SafeActionClient(ActionClient):
    """ActionClient subclass that guards creation and teardown races.

    1. **Creation race** – rclpy's ``ActionClient.__init__`` registers the
       client as a waitable (visible to the executor) *before* assigning
       ``self._lock``.  Pre-initializing ``_lock`` closes this window.

    2. **Destruction race** – when the manager's ``shutdown()`` destroys
       clients, the executor may still hold a reference in its snapshot of
       waitables and crash on the next poll.  The guarded methods return
       zeros/noop if the underlying handle was already invalidated.
    """

    def __init__(self, *args, **kwargs):
        self._lock = threading.Lock()
        super().__init__(*args, **kwargs)

    def get_num_entities(self) -> NumberOfEntities:
        try:
            return super().get_num_entities()
        except (InvalidHandle, RCLError) as e:
            _logger.debug('_SafeActionClient.get_num_entities skipped: %s', e)
            return NumberOfEntities(0, 0, 0, 0, 0, 0)

    def add_to_wait_set(self, wait_set) -> None:
        try:
            super().add_to_wait_set(wait_set)
        except (InvalidHandle, RCLError) as e:
            _logger.debug('_SafeActionClient.add_to_wait_set skipped: %s', e)


class SubscriptionToken:
    """Opaque handle returned by :meth:`RosClientManager.subscribe`.

    Used to cancel a single fan-out listener via
    :meth:`RosClientManager.unsubscribe` without affecting other listeners
    that share the same underlying rclpy subscription.
    """

    __slots__ = ('key', 'callback_id')

    def __init__(self, key: tuple, callback_id: str):
        self.key = key
        self.callback_id = callback_id


class _TopicFanOut:
    """One rclpy subscription dispatched to N registered callbacks."""

    __slots__ = ('subscription', 'callbacks')

    def __init__(self, subscription: Subscription):
        self.subscription: Subscription = subscription
        self.callbacks: dict[str, Callable] = {}


class RosClientManager:
    """Shared, de-duplicated registry of ROS endpoints for a node.

    BT nodes typically create an action/service client, a publisher, or a
    subscription on the BT runner's ROS node.  When those BT nodes are
    recreated (e.g. on every tick of a retry branch) the endpoints leak,
    because rclpy keeps them alive on the node.  Over time this grows the
    executor's wait set and slows down every callback dispatch.

    The manager hands out a single cached endpoint per logical key and keeps
    ownership for the lifetime of the ROS node.  Callers must **not** destroy
    returned objects; teardown happens once in :meth:`shutdown`.

    Subscriptions are multiplexed: one underlying rclpy subscription per
    ``(msg_type, topic_name, qos)`` fans out to all registered callbacks.
    """

    def __init__(self, node: Node):
        self._node = node
        self._lock = threading.Lock()
        self._action_clients: dict[tuple, ActionClient] = {}
        self._service_clients: dict[tuple, Client] = {}
        self._publishers: dict[tuple, Publisher] = {}
        self._subscriptions: dict[tuple, _TopicFanOut] = {}

    # ------------------------------------------------------------------
    # Keying helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _qos_key(qos: Any) -> Any:
        if isinstance(qos, QoSProfile):
            return (qos.reliability, qos.durability, qos.history, qos.depth)
        return qos

    # ------------------------------------------------------------------
    # Action clients
    # ------------------------------------------------------------------
    def get_or_create_action_client(
        self, action_type, action_name: str
    ) -> ActionClient:
        key = (action_type, action_name)
        with self._lock:
            client = self._action_clients.get(key)
            if client is None:
                client = _SafeActionClient(self._node, action_type, action_name)
                self._action_clients[key] = client
            else:
                self._node.get_logger().debug(f"client already exists: {key}")
        return client

    # ------------------------------------------------------------------
    # Service clients
    # ------------------------------------------------------------------
    def get_or_create_service_client(
        self, srv_type, service_name: str
    ) -> Client:
        key = (srv_type, service_name)
        with self._lock:
            client = self._service_clients.get(key)
            if client is None:
                client = self._node.create_client(srv_type, service_name)
                self._service_clients[key] = client
            else:
                self._node.get_logger().debug(f"client already exists: {key}")
        return client

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------
    def get_or_create_publisher(
        self, msg_type, topic_name: str, qos: Any = 10
    ) -> Publisher:
        key = (msg_type, topic_name, self._qos_key(qos))
        with self._lock:
            pub = self._publishers.get(key)
            if pub is None:
                pub = self._node.create_publisher(msg_type, topic_name, qos)
                self._publishers[key] = pub
            else:
                self._node.get_logger().debug(f"publisher already exists: {key}")
        return pub

    # ------------------------------------------------------------------
    # Subscriptions (fan-out)
    # ------------------------------------------------------------------
    def subscribe(
        self,
        msg_type,
        topic_name: str,
        callback: Callable,
        qos: Any = 10,
    ) -> SubscriptionToken:
        key = (msg_type, topic_name, self._qos_key(qos))
        callback_id = uuid4().hex
        with self._lock:
            fanout = self._subscriptions.get(key)
            if fanout is None:
                sub = self._node.create_subscription(
                    msg_type,
                    topic_name,
                    lambda msg, k=key: self._dispatch(k, msg),
                    qos,
                )
                fanout = _TopicFanOut(sub)
                self._subscriptions[key] = fanout
            else:
                self._node.get_logger().debug(f"subscription already exists: {key}")
            
            fanout.callbacks[callback_id] = callback
        return SubscriptionToken(key, callback_id)

    def unsubscribe(self, token: SubscriptionToken) -> None:
        with self._lock:
            fanout = self._subscriptions.get(token.key)
            if fanout is None:
                return
            fanout.callbacks.pop(token.callback_id, None)

    def _dispatch(self, key: tuple, msg) -> None:
        with self._lock:
            fanout = self._subscriptions.get(key)
            callbacks = list(fanout.callbacks.values()) if fanout else []
        for cb in callbacks:
            try:
                cb(msg)
            except Exception as e:
                self._node.get_logger().error(
                    f'RosClientManager: subscription callback on '
                    f'{key[1]!r} raised: {e!r}')

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def shutdown(self) -> None:
        with self._lock:
            action_clients = list(self._action_clients.values())
            service_clients = list(self._service_clients.values())
            publishers = list(self._publishers.values())
            fanouts = list(self._subscriptions.values())
            self._action_clients.clear()
            self._service_clients.clear()
            self._publishers.clear()
            self._subscriptions.clear()

        for client in action_clients:
            try:
                client.destroy()
            except (InvalidHandle, RCLError) as e:
                _logger.debug('action_client.destroy skipped: %s', e)
        for client in service_clients:
            try:
                self._node.destroy_client(client)
            except (InvalidHandle, RCLError) as e:
                _logger.debug('destroy_client skipped: %s', e)
        for pub in publishers:
            try:
                self._node.destroy_publisher(pub)
            except (InvalidHandle, RCLError) as e:
                _logger.debug('destroy_publisher skipped: %s', e)
        for fanout in fanouts:
            try:
                self._node.destroy_subscription(fanout.subscription)
            except (InvalidHandle, RCLError) as e:
                _logger.debug('destroy_subscription skipped: %s', e)
