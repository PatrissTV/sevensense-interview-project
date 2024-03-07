import rclpy
from rclpy.node import Node
import numpy as np
from irobot_create_msgs.action import NavigateToPosition
from nav_msgs.msg import Odometry
import random

from .action_handler import ActionHandler
from .data import States, Flags
from .iRobot import iRobot


class Alphasense(iRobot):
    """Extends the iRobot class to implement the specific behavior of the hypothetical Alphasense Autonomy.

    Attributes:
        node (rclpy.node.Node): The ROS2 node for the Alphasense robot.
        namespace (str): The namespace of the Alphasense robot.
        navigate_to_position (ActionHandler): The action client for the "NavigateToPosition" action.
        odom (rclpy.subscription.Subscription): The subscriber for the odometry topic.
        rival_namespace (str): The namespace of the rival robot.
        rival_state_x (float): The x-coordinate of the rival robot's state.
        rival_state_y (float): The y-coordinate of the rival robot's state.
        collision_prevented (bool): Flag indicating if collision is prevented.
        rival_offset_x (float): The x-offset between the Alphasense robot and the rival robot.
        rival_offset_y (float): The y-offset between the Alphasense robot and the rival robot.
        rival_odom (rclpy.subscription.Subscription): The subscriber for the rival robot's odometry topic.
        go_home_timer (rclpy.timer.Timer): The timer for the go home procedure.
        collision_timer (rclpy.timer.Timer): The timer for verifying collision.
        state_x (float): The x-coordinate of the Alphasense robot's state.
        state_y (float): The y-coordinate of the Alphasense robot's state.
        state_quat (Quaternion): The quaternion representing the Alphasense robot's state orientation.
        dock_x (float): The x-coordinate of the dock position.
        dock_y (float): The y-coordinate of the dock position.
        dock_quat (Quaternion): The quaternion representing the dock position.
        target_x (float): The x-coordinate of the target position.
        target_y (float): The y-coordinate of the target position.
        target_quat (Quaternion): The quaternion representing the target orientation.
        currentState (States): The current state of the Alphasense robot.
        params (dict): A dictionary containing various parameters for the Alphasense robot.

    Methods:
        __init__(self): Initializes the Alphasense robot.
        odom_callback(self, msg): Callback function for the odometry message.
        init_rival(self): Initializes the rival robot by setting the necessary attributes and subscribing to the rival's odometry topic.
        rival_odom_callback(self, msg): Callback function for the rival odometry message.
        verify_collision(self): Verifies if a collision is imminent and takes appropriate actions.
        verify_dynamic_collision(self, x, y): Verifies if random paths are entering the rival's safe space when both robots are close.
        set_target(self, x, y, quat=None): Sets the target position for the Alphasense robot.
        drive_to_target(self): Drives the robot to the target position.
        drive_random(self): Drives the robot to a random position.
        go_home(self): Performs the go home procedure for the robot.
        state_machine(self, flag=Flags.OK): Executes the state machine logic based on the current state and flag.
    """

    def __init__(self):
        """
        Initializes the Alphasense robot.
        """

        self.node = rclpy.create_node('alphasense')
        self.namespace = self.node.get_namespace() if self.node.get_namespace() != '/' else ""
        self.node.get_logger().info(f'Alphasense robot initialized with namespace: {self.namespace}')

        super().__init__()

        # Action clients
        self.navigate_to_position = ActionHandler("NavigateToPosition",self.node, NavigateToPosition, f'{self.namespace}/navigate_to_position',self.state_machine)

        # Subscribers
        self.odom = self.node.create_subscription(Odometry,f'{self.namespace}/odom', self.odom_callback, 10)
        
        # Rival 
        self.init_rival()

        # Timers
        self.go_home_timer = self.node.create_timer(1, self.go_home)
        self.collision_timer = self.node.create_timer(0.1, self.verify_collision) # Verify collision @ 10Hz
        
        # Set seed
        seed = int(self.params['RANDOM_SEED'])
        seed += 100 if self.namespace == "/alpha2" else 0
        self.node.get_logger().info(f'Seed: {seed}, Namespace: {self.namespace}')
        random.seed(seed)

        # Execute state machine
        self.state_machine()

    def odom_callback(self, msg):
        """
        Callback function for the odometry message.

        This function is called whenever a new odometry message is received.
        It saves the dock position from the first odometry message and updates
        the current pose.

        Args:
            msg (Odometry): The odometry message containing the pose information.
        """
        
        # Save dock state from first odom message
        if not self.dock_x or not self.dock_y or not self.dock_quat:
            self.node.get_logger().info(f'Saving dock position | x: {msg.pose.pose.position.x}, y: {msg.pose.pose.position.y}, quat: {msg.pose.pose.orientation}')
            self.dock_x = msg.pose.pose.position.x
            self.dock_y = msg.pose.pose.position.y
            self.dock_quat = msg.pose.pose.orientation

            self.state_machine()

        # Update current pose
        self.state_x = msg.pose.pose.position.x
        self.state_y = msg.pose.pose.position.y
        self.state_quat = msg.pose.pose.orientation

    def init_rival(self):
        """
        Initializes the rival robot by setting the necessary attributes and subscribing to the rival's odometry topic.

        This method sets the rival_namespace based on the current namespace, initializes the rival_state_x and rival_state_y to None,
        sets the collision_prevented flag to False, and calculates the rival_offset_x and rival_offset_y based on the current namespace.
        Finally, it creates a subscription to the rival's odometry topic using the rival_namespace.
        """

        self.rival_namespace = "/alpha1" if self.namespace == "/alpha2" else "/alpha2"
        self.rival_state_x = None
        self.rival_state_y = None
        self.collision_prevented = False

        if self.namespace == "/alpha2":
            self.rival_offset_x = self.params['RIVAL_OFFSET_X']
            self.rival_offset_y = - self.params['RIVAL_OFFSET_Y']
        else:
            self.rival_offset_x = self.params['RIVAL_OFFSET_X']
            self.rival_offset_y = self.params['RIVAL_OFFSET_Y']
            
        self.rival_odom = self.node.create_subscription(Odometry,f'{self.rival_namespace}/odom', self.rival_odom_callback, 10)

    def rival_odom_callback(self, msg):
        """
        Callback function for the rival odometry message.

        This function is called whenever a new rival odometry message is received.
        It updates the rival state x and y coordinates based on the received message.

        Args:
            msg (Odometry): The rival odometry message.
        """

        self.rival_state_x = msg.pose.pose.position.x + self.rival_offset_x
        self.rival_state_y = msg.pose.pose.position.y + self.rival_offset_y

    def verify_collision(self):
        """
        Verifies if a collision is imminent and takes appropriate actions.

        This method updates the collision radius due to covariance and checks if a collision is imminent.
        If a collision is detected, it changes the current state to STOP, cancels the navigation goal, and returns.
        If the current state is STOPPED, it changes the current state to NAVIGATE and calls the state_machine method.
        """

        # Update collision radius due to covariance
        self.params['COLLISION_RADIUS'] += self.params['COLLISION_RADIUS_COVARIANCE']
        if not self.collision_prevented:
            if self.currentState == States.NAVIGATE or self.currentState == States.GOHOME:
                
                dist = np.sqrt((self.state_x - self.rival_state_x)**2 + (self.state_y - self.rival_state_y)**2)
                if dist**2 <= (self.params['COLLISION_RADIUS'])**2:
                    self.node.get_logger().info('Imminent collision detected!')

                    # If the random path is entering the rival's safe space, take action to prevent collision
                    if not self.verify_dynamic_collision(self.target_x, self.target_y):
                        self.node.get_logger().info('Taking action to prevent collision.')
                        self.currentState = States.STOP
                        self.navigate_to_position.cancel_goal_async()
                        return
            
        if self.currentState == States.STOPPED:
            self.node.get_logger().info('Move again!')
            self.currentState = States.NAVIGATE
            self.state_machine()
            
    def verify_dynamic_collision(self, x, y):
        """
        Verifies if random paths are entering the rival's safe space when both robots are close.

        Args:
            x (float): The x-coordinate of the target position.
            y (float): The y-coordinate of the target position.

        Returns:
            bool: True if the collision is prevented or False if the collision is not prevented.
        """

        if (self.state_x - self.rival_state_x)**2 + (self.state_y - self.rival_state_y)**2 < (self.params['COLLISION_RADIUS'])**2:
            target_ego_vec = np.array([x, y]) - np.array([self.state_x, self.state_y])
            rival_ego_vec = np.array([self.rival_state_x, self.rival_state_y]) - np.array([self.state_x, self.state_y])
            
            if np.dot(target_ego_vec, rival_ego_vec) >= 0:
                return False
            else:
                self.node.get_logger().info(f'Planned collision avoidance | target_x {x}, target_y {y} state_x {self.state_x}, state_y {self.state_y}, rival_x {self.rival_state_x}, rival_y {self.rival_state_y}')
                self.collision_prevented = True
                return True
        else:
            self.collision_prevented = False
            return True

    def set_target(self, x, y, quat=None):
        """
        Sets the target position for the AlphaSense object.

        Args:
            x (float): The x-coordinate of the target position.
            y (float): The y-coordinate of the target position.
            quat (Optional): The quaternion representing the target orientation. Defaults to None.

        Returns:
            None
        """

        self.node.get_logger().info(f'Setting target position | x: {x}, y: {y}, quat: {quat}')
        self.target_x = float(x)
        self.target_y = float(y)
        self.target_quat = quat

    def drive_to_target(self):
        """
        Drives the robot to the target position.

        This method sends a navigation goal position to the robot's navigation system.
        The target position is specified by the `target_x` and `target_y` attributes.
        If a target quaternion (`target_quat`) is provided, the robot will also try to achieve the goal heading.
        """
        
        self.node.get_logger().info(f'Sending navigation goal position | x: {self.target_x}, y: {self.target_y}, quat: {self.target_quat}')

        goal = NavigateToPosition.Goal()

        if self.target_quat is not None:
            self.node.get_logger().info(f'Got quat!')
            goal.achieve_goal_heading = True
            goal.goal_pose.pose.orientation = self.target_quat
        else:
            goal.achieve_goal_heading = False

        goal.goal_pose.pose.position.x = self.target_x
        goal.goal_pose.pose.position.y = self.target_y

        self.navigate_to_position.send_goal_async(goal)

    def drive_random(self):
            """
            Drives the robot to a random location within a specified range.

            This method generates a random point within a specified range and drives the robot to that point.
            It verifies if the generated point is valid by checking for static and dynamic collisions.
            If the generated point is not valid after a certain number of iterations, it logs an error message and returns.
            """

            valid = False
            iters = 0
            while not valid:
                if iters > 100:
                    self.node.get_logger().info(f'Problem infeasible!')
                    return

                # Sample a vector with min and max length, and random direction
                radius = random.uniform(self.params['MIN_RAND_DIST'], self.params['MAX_RAND_DIST'])
                angle = random.uniform(0, 2 * np.pi)
                x = self.state_x + radius * np.cos(angle)
                y = self.state_y + radius * np.sin(angle)

                # Recompute random point if point lies in dock area or outside of the map
                valid = self.verify_static_collision(x, y) and self.verify_dynamic_collision(x,y)

            
            self.set_target(x, y)
            self.drive_to_target()

    def go_home(self):
        """
        Performs the go home procedure for the robot.

        Only attempts to go home if the robot is currently in the NAVIGATE state.
        Calculates the maximum feasible distance to the docking station to reach it on time.
        If there is no collision danger , sets the docking interception point as the target and cancels the ongoing navigation action.
        """

        # Only attempt to go home if navigating
        if self.currentState == States.NAVIGATE:
            # Calculate maximum feasible distance to the docking station
            current_dist = np.sqrt((self.state_x - self.dock_x) ** 2 + (self.state_y - self.dock_y) ** 2)
            t_elapsed = self.current_time() - self.start_time
            max_dist = self.params['CRUISE_VELOCITY'] * (self.params['MISSION_TIME'] - t_elapsed) - self.params['MAX_DIST_MARGIN']  # subtract some margin value

            # Checks if we can still get to the docking station on time
            if current_dist >= max_dist:

                x = self.dock_x + self.params['DOCKING_OFFSET_X']
                y = self.dock_y + self.params['DOCKING_OFFSET_Y']

                # If robots are not close enough, we can go home. Otherwise, we continue with random and
                # safe navigation to resolve the conflict
                if self.verify_dynamic_collision(x,y):
                    # Cancel the ongoing navigation action
                    self.currentState = States.GOHOME
                    self.node.get_logger().info(f'Starting go home procedure')

                    # Set the docking interception point as the target
                    self.set_target(x, y, quat=self.dock_quat)
                    self.navigate_to_position.cancel_goal_async()

    def state_machine(self, flag=Flags.OK):
        """
        Executes the state machine logic based on the current state and flag.

        Args:
            flag (Flags): The flag indicating the current state of the system. Defaults to Flags.OK.
        """

        self.currentFlag = flag

        if self.currentState == States.UNINITIALIZED:
            if self.dock_x is None or self.dock_y is None:
                self.node.get_logger().info('Waiting for dock position')
            else:
                self.undock()

        elif self.currentState == States.UNDOCKING:
            if self.currentFlag == Flags.ERROR:
                self.node.get_logger().info('Undocking failed. Undocking manually')
                x = self.dock_x + self.params['UNDOCK_MANUAL_X']
                y = self.dock_y + self.params['UNDOCK_MANUAL_Y']
                self.set_target(x, y)
                self.drive_to_target()
            else:
                self.currentState = States.UNDOCKED
                self.node.get_logger().info(f'Undocked correctly | x: {self.state_x}, y: {self.state_y}')
                self.state_machine()

        elif self.currentState == States.UNDOCKED:
            self.currentState = States.NAVIGATE
            self.start_time = self.current_time()
            self.state_machine()

        elif self.currentState == States.NAVIGATE:
            if self.currentFlag == Flags.OK:
                self.node.get_logger().info(f'Start navigating :) | x: {self.target_x}, y: {self.target_y}')
                self.drive_random()  # Create random target
            else:
                self.currentState = States.STOPPED
                self.node.get_logger().info('Navigation failed')

        elif self.currentState == States.GOHOME:
            self.currentState = States.DOCKING
            self.node.get_logger().info(f'Going home | x: {self.target_x}, y: {self.target_y}')
            self.drive_to_target()

        elif self.currentState == States.DOCKING:
            self.node.get_logger().info('Trying to dock! Almost there!')
            self.dock()

        elif self.currentState == States.STOP:
            self.currentState = States.STOPPED
            self.state_machine()

        elif self.currentState == States.STOPPED:
            self.node.get_logger().info(f'Stopped. That was close!')

        elif self.currentState == States.DOCKED:
            if self.currentFlag == Flags.OK:
                self.node.get_logger().info(f'Docked! Thanks for cleaning with iRobot :)')
            else:
                # Docking fails usually when a namespace is given to the robot.
                self.node.get_logger().info('Docking failed.')

        else:
            self.node.get_logger().info('Unknown state')


def main():
    rclpy.init()
    alphasense = Alphasense()
    rclpy.spin(alphasense.node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
