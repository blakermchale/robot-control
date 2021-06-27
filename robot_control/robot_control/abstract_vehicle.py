#!/usr/bin/env python
# ROS libraries
from types import FrameType
from airsim.types import Pose
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.qos import QoSProfile
from rclpy.logging import get_logging_severity_from_string
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.clock import Duration
from std_msgs.msg import Header
# ROS interfaces
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Twist, Quaternion
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from robot_control_interfaces.action import FollowWaypoints, GoWaypoint
from robot_control_interfaces.msg import Waypoint
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
        self._check_rate = self.create_rate(20)  # Rate to check conditions in actions

        # Publishers
        self._pub_pose = self.create_publisher(PoseStamped, "pose", 10)
        self._pub_target = self.create_publisher(PointStamped, "target", 10)

        # Subscribers
        self._sub_vel = self.create_subscription(Twist, "cmd/velocity", self._callback_velocity, 1)

        # Services
        self._srv_arm = self.create_service(Trigger, "arm", self._handle_arm)
        self._srv_disarm = self.create_service(Trigger, "disarm", self._handle_disarm)
        self._srv_kill = self.create_service(Trigger, "kill", self._handle_kill)

        # Actions
        self._server_go_waypoint = ActionServer(self, GoWaypoint, "go_waypoint", self._handle_go_waypoint_goal,
            cancel_callback=self._handle_go_waypoint_cancel)
        self._server_follow_waypoints = ActionServer(self, FollowWaypoints, "follow_waypoints", self._handle_follow_waypoints_goal,
            cancel_callback=self._handle_follow_waypoints_cancel)

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
        """Arms vehicle.
        
        Returns:
            bool: Flag indicating successful command.
        """
        raise NotImplementedError

    def disarm(self):
        """Disarms vehicle.
        
        Returns:
            bool: Flag indicating successful command.
        """
        raise NotImplementedError
    
    def kill(self):
        """Kills vehicle.

        Returns:
            bool: Flag indicating successful command.
        """
        raise NotImplementedError

    def set_target(self, x: float = np.nan, y: float = np.nan, z: float = np.nan):
        """Sets internal target variable in local NED for state checking and debugging.

        Args:
            x (float): x position
            y (float): y position
            z (float): z position
        """
        self._target_position = np.asfarray([x, y, z])
    
    def send_waypoint(self, x: float, y: float, z: float, heading: float, frame: int = Waypoint.LOCAL_NED):
        """Sends a waypoint to the vehicle in a specified frame.

        Args:
            x (float): x position meters
            y (float): y position meters
            z (float): z position meters
            frame (int, optional): Enum specifying frame for commands. Defaults to Waypoint.LOCAL_NED.
        """
        raise NotImplementedError

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float, frame: int = Waypoint.LOCAL_NED):
        """Sends a velocity command to the vehicle in a specified frame.

        Args:
            vx (float): x velocity (m/s)
            vy (float): y velocity (m/s)
            vz (float): z velocity (m/s)
            yaw_rate (float): yaw rate (rad/s)
            frame (int, optional): Enum specifying frame for commands. Defaults to Waypoint.LOCAL_NED.
        """
        raise NotImplementedError

    #####################
    ## Checking states
    #####################
    def distance_to_target(self):
        """Returns distance to the target coordinate

        Returns:
            float: Distance in m
        """
        return np.linalg.norm(self._position-self._target_position)

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

    def is_armed(self):
        """Checks if vehicle's motors are armed.

        Returns:
            bool: Whether the vehicle is armed.
        """
        raise NotImplementedError

    ########################
    ## Actions
    ########################
    def _handle_go_waypoint_goal(self, goal: GoWaypoint):
        """Callback for going to a single waypoint."""
        if not self._precheck_go_waypoint_goal():
            goal.abort()
            return GoWaypoint.Result()
        position = goal.request.waypoint.position
        heading = goal.request.waypoint.heading
        frame = goal.request.waypoint.frame
        self.send_waypoint(position.x, position.y, position.z, heading, frame)
        feedback_msg = GoWaypoint.Feedback()
        while True:
            self._check_rate.sleep()
            distance = self.distance_to_target()    
            reached = self.reached_target(distance=distance)     
            if distance is not None:  # publish feedback
                feedback_msg.distance = float(distance)
                goal.publish_feedback(feedback_msg)
            if goal.is_cancel_requested:
                goal.canceled()  #handle cancel action
                return GoWaypoint.Result()
            if reached:
                self.get_logger().info("GoWaypoint: reached destination")
                goal.succeed()  #handle success
                return GoWaypoint.Result()

    def _precheck_go_waypoint_goal(self):
        """Performs prechecks to see if vehicle can go to waypoint."""
        if not self.is_armed():
            self.get_logger().warn("GoWaypoint: aborted not armed")
            return False
        return True

    def _handle_go_waypoint_cancel(self, cancel):
        """Callback for cancelling waypoint."""
        self.halt()
        return CancelResponse.ACCEPT

    def _handle_follow_waypoints_goal(self, goal: FollowWaypoints):
        """Callback for following a path of waypoints."""
        raise NotImplementedError

    def _handle_follow_waypoints_cancel(self, cancel):
        """Callback for cancelling path."""
        raise NotImplementedError

    ########################
    ## Services
    ########################
    def _handle_arm(self, req, res):
        """Callback for arming vehicle."""
        res.success = self.arm()
        return res

    def _handle_disarm(self, req, res):
        """Callback for disarming vehicle."""
        res.success = self.disarm()
        return res

    def _handle_kill(self, req, res):
        """Callback for killing vehicle."""
        res.success = self.kill()
        return res

    ########################
    ## Subscribers
    ########################
    def _callback_velocity(self, msg: Twist):
        """Callback for receiving velocity commands to send to vehicle."""
        v = msg.linear
        yaw_rate = msg.angular.z
        self.send_velocity(v.x, v.y, v.z, yaw_rate, Waypoint.LOCAL_NED)

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
