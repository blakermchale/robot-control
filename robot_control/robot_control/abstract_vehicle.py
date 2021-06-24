#!/usr/bin/env python
# ROS libraries
from airsim.types import Pose
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.qos import QoSProfile
from rclpy.logging import get_logging_severity_from_string
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.clock import Duration
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Twist, Quaternion
from nav_msgs.msg import Odometry
# Common libraries
import numpy as np
from scipy.spatial.transform import Rotation as R


class AVehicle(Node):
    def __init__(self, log_level="info", instance=0):
        super().__init__('vehicle') # start node
        if log_level is not None:
            self.get_logger().set_level(get_logging_severity_from_string(log_level))
        self._default_callback_group = ReentrantCallbackGroup()  # ROS processes need to be run in parallel for this use case
        self.instance = instance
        self.namespace = self.get_namespace().split("/")[-1]
        self._position = np.asfarray([np.nan, np.nan, np.nan])
        self._orientation = np.asfarray([np.nan, np.nan, np.nan, np.nan])
        self._target_position = np.asfarray([np.nan, np.nan, np.nan])

        # Setup loop
        timer_period = 1/20  # seconds
        self._timer_update = self.create_timer(timer_period, self.update)

        # Publishers
        self._pub_pose = self.create_publisher(PoseStamped, "pose", 10)
        self._pub_target = self.create_publisher(PointStamped, "target", 10)

        # Subscribers
        self._sub_vel = self.create_subscription(Twist, "cmd/velocity", self._callback_velocity, 1)

        # ROS parameters
        self.declare_parameter('tolerance_location', 0.7)
        print("AVehicle")

    def update(self):
        """Main loop for performing vehicle checks and updates.
        
        Should set `self._position`, `self._orientation`.
        """
        self._publish_pose()
        self._publish_target()

    ###################
    ## Control commands
    ###################
    def halt(self):
        """Halts vehicle at current location"""
        raise NotImplementedError

    def arm(self):
        """Arms vehicle."""
        raise NotImplementedError

    def disarm(self):
        """Disarms vehicle."""
        raise NotImplementedError

    def set_target(self, x: float = np.nan, y: float = np.nan, z: float = np.nan):
        """[summary]

        Args:
            x (float): x position
            y (float): y position
            z (float): z position
        """
        self._target_position = np.asfarray([x, y, z])

    #####################
    ## Checking states
    #####################
    def distance_to_target(self):
        """Returns distance to the target coordinate

        Returns:
            float: Distance in m
        """
        raise NotImplementedError

    def reached_target(self, tolerance=None, distance=None):
        """Determines if we have reached the target

        Args:
            tolerance (float, optional): Metres of tolerance to say that a target has been reached. Defaults to `tolerance_location` ros2 parameter.
            distance (float, optional): Distance away from target. Defaults to vehicle distance away.

        Returns:
            bool: Whether the target has been reached
        """
        if tolerance is None:
            tolerance = self.get_parameter('tolerance_location').value
        return tolerance >= (self.distance_to_target() if distance is None else distance)

    ########################
    ## Subscribers
    ########################
    def _callback_velocity(self, msg):
        """Callback for receiving velocity commands to send to vehicle."""
        raise NotImplementedError

    ########################
    ## Publishers
    ########################
    def _publish_telemetry(self):
        """Publishes common information about vehicle."""
        raise NotImplementedError

    def _publish_target(self):
        """Publishes target point."""
        msg = PointStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = f"world"  # TODO: update frame
        msg.header = header
        msg.point = get_point_from_ned(self._target_position[0], self._target_position[1], self._target_position[2])
        self._pub_target.publish(msg)

    def _publish_pose(self):
        """Publishes pose for debugging."""
        msg = PoseStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "world"  # TODO: update frame
        msg.header = header
        msg.pose.position = get_point_from_ned(self._position[0], self._position[1], self._position[2])
        # r = R.from_quat(self._orientation)  # TODO: separate quat math to helper
        # euler = r.as_euler('xyz')
        # euler[2] = -euler[2]
        q = self._orientation  # R.from_euler('xyz', euler).as_quat()
        msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self._pub_pose.publish(msg)

    def _publish_state(self):
        """Publishes relevant state information. Implementation varies between vehicles."""
        raise NotImplementedError

def get_point_from_ned(north: float, east: float, down: float):
    """Converts NED to ROS Point message.

    Args:
        north (float): north in meters
        east (float): east in meters
        down (float): down in meters
    """
    point = Point()
    point.x = float(north)
    point.y = float(-1.0*east)
    point.z = float(-1.0*down)
    return point
