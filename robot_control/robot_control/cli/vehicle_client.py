#!/usr/bin/env python3
'''
vehicle_client.py
=======================

Generic vehicle client to ROS2 actions and topics in companion_computing. Easy publishing, calling,
and sending goals.
'''
from typing import List
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from robot_control_interfaces.msg import Waypoint
from robot_control_interfaces.action import FollowWaypoints, GoWaypoint

import functools


class VehicleClient(Node):
    def __init__(self, namespace=None):
        super().__init__("vehicle_client", namespace=namespace)
        self._default_callback_group = ReentrantCallbackGroup()
        self._sub_pose = self.create_subscription(PoseStamped, "pose", self._cb_pose, 10)
        self._cli_go_to_coordinate = ActionClient(self, GoWaypoint, "go_waypoint")
        self._cli_follow_coordinates = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self._pose = PoseStamped()
        self._poses_received = 0
        self._timeout_sec = 60.0

        # Goal handles
        self._goal_handles = {}

    def send_go_waypoint(self, x: float, y: float, z: float, heading: float, frame):
        self.reset()
        if not self._cli_go_to_coordinate.wait_for_server(timeout_sec=self._timeout_sec):
            self.get_logger().error("No action server available")
            return
        goal = GoWaypoint.Goal()
        goal.waypoint.frame = frame
        goal.waypoint.heading = heading
        goal.waypoint.position.x = x
        goal.waypoint.position.y = y
        goal.waypoint.position.z = z
        self.get_logger().info("Sending goal to `go_waypoint`")
        future = self._cli_go_to_coordinate.send_goal_async(goal)
        future.add_done_callback(functools.partial(self._action_response, "go_waypoint"))
        return future

    def send_follow_waypoints(self, coordinates: List[Waypoint], tolerance: float):
        self.reset()
        if not self._cli_follow_coordinates.wait_for_server(timeout_sec=self._timeout_sec):
            self.get_logger().error("No action server available")
            return
        goal = FollowWaypoints.Goal()
        goal.coordinates = coordinates
        goal.tolerance = tolerance
        self.get_logger().info(f"Sending goal to `follow_waypoints`")
        future = self._cli_follow_coordinates.send_goal_async(goal, feedback_callback=self._feedback_follow_coordinates)
        future.add_done_callback(functools.partial(self._action_response, "follow_waypoints"))
        return future

    ########################
    ## Helpers
    ########################
    def reset(self):
        """Resets state of client."""
        self._goal_handles = {}

    def _action_response(self, action_name: str, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected for '{action_name}'")
            return
        self.get_logger().info(f"Goal accepted for '{action_name}'")
        self._goal_handles[action_name] = goal_handle

    ########################
    ## Subscribers
    ########################
    def _cb_pose(self, msg: PoseStamped):
        self._pose = msg
        self._poses_received += 1

    #######################
    ## Feedback callbacks
    #######################
    def _feedback_follow_coordinates(self, feedback):
        self.get_logger().info(f"`follow_waypoints` feedback: {feedback.feedback.distance}m", throttle_duration_sec=2.0)
