from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from .data import States, Flags

class ActionHandler:
    """
    A class that handles actions for a specific node.

    Args:
        name (str): The name of the action handler.
        node (Node): The ROS2 node associated with the action handler.
        action (str): The action type.
        topic (str): The topic to publish the action to.
        parent_callback (function): The callback function to be called by the parent.

    Attributes:
        name (str): The name of the action handler.
        node (Node): The ROS2 node associated with the action handler.
        parent_callback (function): The callback function to be called by the parent.
        action (ActionClient): The action client used to send goals.
        goal_handle (GoalHandle): The handle for the current goal.
        result_future (Future): The future object for retrieving the result of the goal.
    """

    def __init__(self, name, node, action, topic, parent_callback):

        self.name = name
        self.node = node
        self.parent_callback = parent_callback
        self.action = ActionClient(self.node, action, topic)
        while not self.action.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().info(f'Waiting for {self.name} action server...')

    def send_goal_async(self, goal):
        """
        Sends a goal asynchronously.

        Args:
            goal: The goal to be sent.

        """
        future = self.action.send_goal_async(goal)
        future.add_done_callback(self.action_callback)

    def cancel_goal_async(self):
        """
        Cancels the current goal asynchronously.

        """
        self.goal_handle.cancel_goal_async()

    def action_callback(self, future, action=None):
        """
        Callback function for the action.

        Args:
            future: The future object for the action.
            action: The action associated with the callback.

        """
        handle = future.result()
        if not handle.accepted:
            self.node.get_logger().info(f'{self.name} Goal rejected')
            self.parent_callback(Flags.ERROR)
            return

        self.node.get_logger().info(f'{self.name} accepted')

        # Save handle in case we want to cancel it to avoid collision or go home
        self.goal_handle = future.result()

        self.result_future = handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future, action=None):
        """
        Callback function for the result of the action.

        Args:
            future: The future object for the result.
            action: The action associated with the callback.

        """
        result = future.result().result
        status = future.result().status

        flag = Flags.ERROR
        # OK only if succeeded and not successfully canceled
        if status == GoalStatus.STATUS_SUCCEEDED:
            flag = Flags.OK

        self.node.get_logger().info(f'Action {action} ended with result {result} and status {status}')
        self.parent_callback(flag)

    