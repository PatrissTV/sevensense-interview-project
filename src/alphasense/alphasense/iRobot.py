import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.action import ActionClient
from irobot_create_msgs.action import Undock, NavigateToPosition, Dock
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.clock import Clock

import random

from .action_handler import ActionHandler
from .data import States, Flags

import time


class iRobot():
    """A class representing the inherent properties and actions of an iRobot.
    Requires a ROS2 node to be initialized in child class.

    Attributes:
        state_x (float): The current x-coordinate of the robot's state.
        state_y (float): The current y-coordinate of the robot's state.
        target_x (float): The target x-coordinate for the robot's movement.
        target_y (float): The target y-coordinate for the robot's movement.
        target_quat (float): The target quaternion for the robot's orientation.
        dock_x (float): The x-coordinate of the docking station.
        dock_y (float): The y-coordinate of the docking station.
        dock_quat (float): The quaternion of the docking station's orientation.
        currentState (States): The current state of the robot.
        currentFlag (Flags): The current flag status of the robot.
        undock_client (ActionHandler): The action client for the "Undock" action.
        dock_client (ActionHandler): The action client for the "Dock" action.
        params (dict): A dictionary containing the parameters retrieved from the ROS node.

    Methods:
        __init__(self, robot_name=""): Initializes a new instance of the iRobot class.
        get_params(self): Retrieves the parameters from the ROS node and stores them in a dictionary.
        current_time(self): Returns the current time in seconds.
        verify_static_collision(self, x, y): Verifies if the given coordinates are within the playground boundaries.
        undock(self): Sends an undock request to the robot.
        dock(self): Sends a dock request to the robot.
    """
       
    def __init__(self):
            """
            Initializes an instance of the iRobot class, it's attributes and action handlers.
            """
            #current states
            self.state_x = 0.0
            self.state_y = 0.0

            # target states
            self.target_x = None
            self.target_y = None
            self.target_quat = None

            # dock state
            self.dock_x = None
            self.dock_y = None
            self.dock_quat = None

            # Set initial state
            self.currentState = States.UNINITIALIZED
            self.currentFlag = Flags.OK

            # Get parameters
            self.get_params()

            # Action clients
            self.undock_client = ActionHandler("Undock",self.node, Undock, f'{self.namespace}/undock',self.state_machine)
            self.dock_client = ActionHandler("Dock",self.node, Dock, f'{self.namespace}/dock',self.state_machine)

    def get_params(self):
        """
        Retrieves the parameters from the ROS node and stores them in a dictionary.
        """

        self.node.declare_parameter("MAX_RAND_DIST")
        self.node.declare_parameter("MIN_RAND_DIST")
        self.node.declare_parameter("DOCKING_OFFSET_X")
        self.node.declare_parameter("DOCKING_OFFSET_Y")
        self.node.declare_parameter("UNDOCK_MANUAL_X")
        self.node.declare_parameter("UNDOCK_MANUAL_Y")
        self.node.declare_parameter("MAX_DIST_MARGIN")
        self.node.declare_parameter("CRUISE_VELOCITY")
        self.node.declare_parameter("MISSION_TIME")
        self.node.declare_parameter("COLLISION_RADIUS")
        self.node.declare_parameter("COLLISION_RADIUS_COVARIANCE")
        self.node.declare_parameter("RIVAL_OFFSET_X")
        self.node.declare_parameter("RIVAL_OFFSET_Y")
        self.node.declare_parameter("RANDOM_SEED")
        self.node.declare_parameter("MAX_X")
        self.node.declare_parameter("MIN_X")
        self.node.declare_parameter("MAX_Y")
        self.node.declare_parameter("MIN_Y")

        self.params = {
            "MAX_RAND_DIST": self.node.get_parameter("MAX_RAND_DIST").value,
            "MIN_RAND_DIST": self.node.get_parameter("MIN_RAND_DIST").value,
            "DOCKING_OFFSET_X": self.node.get_parameter("DOCKING_OFFSET_X").value,
            "DOCKING_OFFSET_Y": self.node.get_parameter("DOCKING_OFFSET_Y").value,
            "UNDOCK_MANUAL_X": self.node.get_parameter("UNDOCK_MANUAL_X").value,
            "UNDOCK_MANUAL_Y": self.node.get_parameter("UNDOCK_MANUAL_Y").value,
            "MAX_DIST_MARGIN": self.node.get_parameter("MAX_DIST_MARGIN").value,
            "CRUISE_VELOCITY": self.node.get_parameter("CRUISE_VELOCITY").value,
            "MISSION_TIME": self.node.get_parameter("MISSION_TIME").value,
            "COLLISION_RADIUS": self.node.get_parameter("COLLISION_RADIUS").value,
            "COLLISION_RADIUS_COVARIANCE": self.node.get_parameter("COLLISION_RADIUS_COVARIANCE").value,
            "RIVAL_OFFSET_X": self.node.get_parameter("RIVAL_OFFSET_X").value,
            "RIVAL_OFFSET_Y": self.node.get_parameter("RIVAL_OFFSET_Y").value,
            "RANDOM_SEED": self.node.get_parameter("RANDOM_SEED").value,
            "MAX_X": self.node.get_parameter("MAX_X").value,
            "MIN_X": self.node.get_parameter("MIN_X").value,
            "MAX_Y": self.node.get_parameter("MAX_Y").value,
            "MIN_Y": self.node.get_parameter("MIN_Y").value
        }

    def current_time(self):
        """
        Returns the current time in seconds. Uses simulation time if defined in parameter file.

        Returns:
            float: The current time in seconds.
        """

        time = self.node.get_clock().now()
        time = time.nanoseconds*1e-9
        return time

    def verify_static_collision(self, x, y):
        """
        Verifies if the given coordinates are within the playground boundaries.

        Parameters:
            x (float): The x-coordinate of the point to be verified.
            y (float): The y-coordinate of the point to be verified.

        Returns:
            bool: True if the point is within the playground boundaries, False otherwise.
        """

        # Convex set defining the playground. Should exclude docking stations.
        if x < self.params['MAX_X'] and x > self.params['MIN_X'] and y < self.params['MAX_Y'] and y > self.params['MIN_Y']:
            return True
        
        return False

    def undock(self):
        """
        Sets the current state to UNDOCKING and sends an undock request to the robot.
        """

        self.currentState = States.UNDOCKING
        self.node.get_logger().info(f'Sending undock request')
        goal = Undock.Goal()
        self.undock_client.send_goal_async(goal)

    def dock(self):
        """
        Sets the current state to DOCKED and sends a dock request to the robot.
        """

        self.currentState = States.DOCKED
        self.node.get_logger().info(f'Sending dock request')
        goal = Dock.Goal()
        self.dock_client.send_goal_async(goal)


