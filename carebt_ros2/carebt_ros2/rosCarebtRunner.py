# Copyright 2022 Andreas Steck (steck.andi@gmail.com)
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

from threading import Thread

from carebt import BehaviorTreeRunner
from carebt import LogLevel
from carebt import TreeNode
from carebt_ros2.rosClientManager import RosClientManager
from carebt_ros2.rosLogger import RosLogger
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

########################################################################


class _BtNode(Thread):

    def __init__(self, ros_node: Node):
        Thread.__init__(self, target=self.thread, daemon=True)
        self.__ros_node = ros_node

        # Use a MultiThreadedExecutor to enable processing goals concurrently
        self.__executor = MultiThreadedExecutor()
        self.__bt_runner = BehaviorTreeRunner()
        self.__bt_runner.set_logger(RosLogger(self.__ros_node.get_logger()))
        self.__bt_runner.get_logger().set_log_level(LogLevel.INFO)

    def thread(self):
        rclpy.spin(self.__ros_node, executor=self.__executor)

    def run_node(self, node: TreeNode, params: str = None):
        self.__bt_runner.node = self.__ros_node
        self.__bt_runner.run(node, params)

    def get_bt_runner(self) -> BehaviorTreeRunner:
        return self.__bt_runner

########################################################################


class RosCarebtRunner(Node):

    def __init__(self, node_name: str = 'carebt_runner'):
        rclpy.init(args=None)
        Node.__init__(self, node_name)

        self.declare_parameter('execution_trace.enabled', False)
        self.declare_parameter('execution_trace.history_depth', 5000)
        self.declare_parameter('execution_trace.max_value_bytes', 1024)
        self.declare_parameter('execution_trace.max_collection_items', 50)
        self.declare_parameter('execution_trace.max_depth', 4)
        self.declare_parameter('execution_trace.snapshot_period_ms', 250)
        self.declare_parameter(
            'execution_trace.redacted_name_patterns',
            ['password', 'passwd', 'secret', 'token', 'api_key'],
        )

        self.client_manager = RosClientManager(self)

        self.__btNode = _BtNode(self)
        self.__btNode.start()
        self.__execution_trace = None
        if self.get_parameter('execution_trace.enabled').value:
            from carebt_ros2.execution_trace import ExecutionTraceConfig
            from carebt_ros2.execution_trace import ExecutionTracePublisher

            config = ExecutionTraceConfig(
                history_depth=self.get_parameter('execution_trace.history_depth').value,
                max_value_bytes=self.get_parameter('execution_trace.max_value_bytes').value,
                max_collection_items=(
                    self.get_parameter('execution_trace.max_collection_items').value),
                max_depth=self.get_parameter('execution_trace.max_depth').value,
                snapshot_period_ms=(
                    self.get_parameter('execution_trace.snapshot_period_ms').value),
                redacted_name_patterns=tuple(
                    self.get_parameter('execution_trace.redacted_name_patterns').value),
            )
            self.__execution_trace = ExecutionTracePublisher(
                self, self.__btNode.get_bt_runner(), config)

    def destroy_node(self) -> bool:
        if self.__execution_trace is not None:
            self.__execution_trace.destroy()
            self.__execution_trace = None
        self.client_manager.shutdown()
        return super().destroy_node()

    def run(self, node: TreeNode, params: str = None) -> None:
        """
        Execute the provided node, respectively the provided behavior tree.

        Parameters
        ----------
        node: TreeNode
            The node which should be executed
        params: str, optional
            The parameters for the node which should be executed

        """
        if self.__execution_trace is not None:
            self.__execution_trace.start_session()
        try:
            self.__btNode.run_node(node, params)
        finally:
            if self.__execution_trace is not None:
                self.__execution_trace.end_session()

    def get_bt_runner(self) -> BehaviorTreeRunner:
        return self.__btNode.get_bt_runner()
