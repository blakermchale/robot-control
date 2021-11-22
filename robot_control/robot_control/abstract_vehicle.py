#!/usr/bin/env python
# ROS libraries
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.clock import Duration
from std_msgs.msg import Header
# ROS interfaces
from geometry_msgs.msg import PointStamped, PoseStamped, Twist, Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from robot_control_interfaces.action import FollowWaypoints, GoWaypoint
from robot_control_interfaces.msg import Waypoint
# Our libraries
from .utils import NpPose, NpTwist, NpVector3, NpVector4, Frame, Axis, AXIS_TO_MASK
from .utils.math import angular_dist
# Common libraries
import numpy as np


class AVehicle(Node):
    def __init__(self, instance=0):
        super().__init__('vehicle') # start node
        self._default_callback_group = ReentrantCallbackGroup()  # ROS processes need to be run in parallel for this use case
        self.instance = instance
        self._namespace = self.get_namespace().split("/")[-1]
        self.pose = NpPose(NpVector3.xyz(0.0, 0.0, 0.0),
            NpVector4.xyzw(0.0, 0.0, 0.0, 1.0))  # NED Pose
        self._lin_vel = NpVector3.xyz(0.0, 0.0, 0.0)
        self._ang_vel = NpVector3.xyz(0.0, 0.0, 0.0)
        self.target = NpPose(NpVector3.xyz(0.0, 0.0, 0.0),
            NpVector4.xyzw(0.0, 0.0, 0.0, 1.0))  # NED Target
        self._wait_moved = Duration(seconds=5)

        # Setup loop
        timer_period = 1/20  # seconds
        self._timer_update = self.create_timer(timer_period, self.update)
        self._check_rate = self.create_rate(20)  # Rate to check conditions in actions

        # Publishers
        self._pub_pose = self.create_publisher(PoseStamped, "pose", 10)
        self._pub_odom = self.create_publisher(Odometry, "odom", 10)
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
        self.declare_parameter('tolerance.xyz', 0.7)
        self.declare_parameter('tolerance.yaw', 0.1)
        self.get_logger().debug("AVehicle initialized")

    def update(self):
        """Main loop for performing vehicle checks and updates.
        
        Should set `self.position`, `self.orientation`.
        """
        self._publish_pose_odom()
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

    def set_target(self, x: float, y: float, z: float, yaw: float, to_ned:bool=False):
        """Sets internal target variable in local NED for state checking and debugging.

        Args:
            x (float): x position (m)
            y (float): y position (m)
            z (float): z position (m)
            yaw (float): heading (rad)
        """
        if to_ned:
            y *= -1
            z *= -1
        self.target.position = [x, y, z]
        self.target.euler.xyz = [0.0, 0.0, yaw]
    
    def send_waypoint(self, x: float, y: float, z: float, heading: float, frame: int = Frame.LOCAL_NED):
        """Sends a waypoint to the vehicle in a specified frame.

        Args:
            x (float): x position (m)
            y (float): y position (m)
            z (float): z position (m)
            frame (int, optional): Enum specifying frame for commands. Defaults to Frame.LOCAL_NED.

        Returns:
            bool: Flag indicating if waypoint was sent successfully.
        """
        raise NotImplementedError

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float, frame: int = Frame.FRD):
        """Sends a velocity command to the vehicle in a specified frame.

        Args:
            vx (float): x velocity (m/s)
            vy (float): y velocity (m/s)
            vz (float): z velocity (m/s)
            yaw_rate (float): yaw rate (rad/s)
            frame (int, optional): Enum specifying frame for commands. Defaults to Frame.FRD.
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
        return np.linalg.norm(self.position-self.target.position)

    def reached_target(self, tolerance=None, distance=None):
        """Determines if we have reached the target

        Args:
            tolerance (float, optional): Metres of tolerance to say that a target has been reached. Defaults to `tolerance.xyz` ros2 parameter.
            distance (float, optional): Distance away from target. Defaults to vehicle distance away.

        Returns:
            bool: Whether the target has been reached
        """
        if tolerance is None:
            tolerance = self.get_parameter('tolerance.xyz').value
        tolerance_yaw = self.get_parameter('tolerance.yaw').value
        # TODO: need to get euler of vehicle in its FRD frame for maps with slants and ground vehicle
        return tolerance >= (self.distance_to_target() if distance is None else distance) and np.abs(angular_dist(self.euler.z, self.target.euler.z)) <= tolerance_yaw

    def has_moved(self, init_position: np.ndarray, axis: Axis = Axis.XYZ):
        """Determines if vehicle has moved significantly.

        Args:
            init_position (np.ndarray): Initial position of vehicle to compare against.
            axis (Axis, optional): Enum representing axis to compare movement against. Defaults to Axis.XYZ

        Returns:
            bool: Whether the vehicle has moved from its initial position.
        """
        mask = AXIS_TO_MASK[axis]
        position = self.position * mask
        init_position *= mask
        dist = np.linalg.norm(position - init_position)
        tolerance = self.get_parameter('tolerance.xyz').value
        # self.get_logger().debug(f"Checking moved from {init_position}, dist: {dist} m", throttle_duration_sec=1.0)
        return dist > tolerance

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
            self._abort_go_waypoint()
            goal.abort()
            return GoWaypoint.Result()
        position = goal.request.waypoint.position
        heading = goal.request.waypoint.heading
        frame = goal.request.waypoint.frame
        self.get_logger().debug(f"GoWaypoint: sending x: {position.x}, y: {position.y}, z: {position.z}, heading: {heading}, frame: {frame}")
        if not self.send_waypoint(position.x, position.y, position.z, heading, frame):
            self._abort_go_waypoint()
            goal.abort()
            return GoWaypoint.Result()
        feedback_msg = GoWaypoint.Feedback()
        start_time = self.get_clock().now()
        init_position = self.position.copy()
        while True:
            if self.get_clock().now() - start_time > self._wait_moved and not self.has_moved(init_position):
                self.get_logger().error("GoWaypoint: hasn't moved aborting")
                goal.abort()
                self._abort_go_waypoint()
                return GoWaypoint.Result()
            distance = self.distance_to_target()    
            reached = self.reached_target(distance=distance)     
            if distance is not None:  # publish feedback
                feedback_msg.distance = float(distance)
                goal.publish_feedback(feedback_msg)
            if goal.is_cancel_requested:
                goal.canceled()  #handle cancel action
                self._abort_go_waypoint()
                return GoWaypoint.Result()
            if reached:
                self.get_logger().info("GoWaypoint: reached destination")
                self._succeed_go_waypoint()
                goal.succeed()  #handle success
                return GoWaypoint.Result()
            self._check_rate.sleep()

    def _precheck_go_waypoint_goal(self):
        """Performs prechecks to see if vehicle can go to waypoint."""
        if not self.is_armed():
            self.get_logger().warn("GoWaypoint: aborted not armed")
            return False
        return True

    def _abort_go_waypoint(self):
        """Called when `go_waypoint` is aborted."""
        pass

    def _succeed_go_waypoint(self):
        """Called when `go_waypoint` succeeds."""
        pass

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
        self.get_logger().debug("Arm service called.")
        res.success = self.arm()
        return res

    def _handle_disarm(self, req, res):
        """Callback for disarming vehicle."""
        self.get_logger().debug("Disarm service called.")
        res.success = self.disarm()
        return res

    def _handle_kill(self, req, res):
        """Callback for killing vehicle."""
        self.get_logger().debug("Kill service called.")
        res.success = self.kill()
        return res

    ########################
    ## Subscribers
    ########################
    def _callback_velocity(self, msg: Twist):
        """Callback for receiving velocity commands to send to vehicle."""
        v = msg.linear
        yaw_rate = msg.angular.z
        self._pre_callback_velocity()
        self.send_velocity(v.x, v.y, v.z, yaw_rate, Frame.FRD)

    def _pre_callback_velocity(self):
        """Gets called prior to velocity being sent."""
        pass

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
        msg.point = get_point_from_ned(self.target.position.get_point_msg())
        self._pub_target.publish(msg)

    def _publish_pose_odom(self):
        """Publishes pose and odometry for debugging."""
        # PoseStamped
        pose_msg = PoseStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "world"  # TODO: update frame
        pose_msg.header = header
        pose = Pose()
        pose.position = get_point_from_ned(self.position.get_point_msg())
        # r = R.from_quat(self.orientation)  # TODO: separate quat math to helper
        # euler = r.as_euler('xyz')
        # euler[2] = -euler[2]
        # q = self.orientation  # R.from_euler('xyz', euler).as_quat()
        pose.orientation = self.orientation.get_quat_msg()
        pose_msg.pose = pose
        # Odometry
        odom_msg = Odometry()
        odom_msg.header = header
        odom_msg.pose.pose = pose
        odom_msg.twist.twist.linear = self.lin_vel.get_vector3_msg()
        odom_msg.twist.twist.angular = self.lin_vel.get_vector3_msg()
        # Publish messages
        self._pub_pose.publish(pose_msg)
        self._pub_odom.publish(odom_msg)

    def _publish_state(self):
        """Publishes relevant state information. Implementation varies between vehicles."""
        raise NotImplementedError

    ####################
    ## Properties
    ####################
    @property
    def position(self):
        """Position in NED"""
        return self.pose.position

    @position.setter
    def position(self, value):
        self.pose.position = self.__get_vector3(value)

    @property
    def orientation(self):
        return self.pose.orientation

    @orientation.setter
    def orientation(self, value):
        self.pose.orientation = self.__get_vector4(value)

    @property
    def euler(self):
        return self.pose.orientation.euler

    @euler.setter
    def euler(self, value):
        self.pose.orientation.euler = value

    @property
    def lin_vel(self):
        return self._lin_vel

    @lin_vel.setter
    def lin_vel(self, value):
        self._lin_vel = self.__get_vector3(value)

    @property
    def ang_vel(self):
        return self._ang_vel

    @ang_vel.setter
    def ang_vel(self, value):
        self._ang_vel = self.__get_vector3(value)

    def __get_vector3(self, value):
        if not isinstance(value, NpVector3):
            return NpVector3(value)
        else:
            return value
    
    def __get_vector4(self, value):
        if not isinstance(value, NpVector4):
            return NpVector4(value)
        else:
            return value


def get_point_from_ned(point):
    """Converts NED ROS Point to ROS Point message.

    Args:
        north (float): north in meters
        east (float): east in meters
        down (float): down in meters
    """
    point.y *= -1
    point.z *= -1
    return point
