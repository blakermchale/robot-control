#!/usr/bin/env python3
'''
drone_client.py
=======================

Generic drone client to ROS2 actions and topics in companion_computing. Easy publishing, calling,
and sending goals.
'''
from rclpy.action import ActionClient

from robot_control_interfaces.action import ArmTakeoff, Land

import functools
from robot_control.cli.vehicle_client import VehicleClient
from rclpy.task import Future


class DroneClient(VehicleClient):
    def __init__(self, namespace=None):
        super().__init__(namespace=namespace)
        self._cli_arm_takeoff = ActionClient(self, ArmTakeoff, "arm_takeoff")
        self._cli_land = ActionClient(self, Land, "land")

    def send_arm_takeoff(self, altitude: float) -> Future:
        """Sends arm and takeoff command to drone.

        Args:
            altitude (float): height to takeoff (m)

        Returns:
            Future: a Future instance to a goal handle that completes when the handle has been
                accepted or rejected
        """
        self.reset()
        if not self._cli_arm_takeoff.wait_for_server(timeout_sec=self._timeout_sec):
            self.get_logger().error("No action server available")
            return
        goal = ArmTakeoff.Goal(altitude=altitude)
        self.get_logger().info(f"Sending goal to `arm_takeoff`")
        future = self._cli_arm_takeoff.send_goal_async(goal, feedback_callback=self._feedback_arm_takeoff)
        future.add_done_callback(functools.partial(self._action_response, "arm_takeoff"))
        return future

    def send_land(self) -> Future:
        """Sends land command to drone.

        Returns:
            Future: a Future instance to a goal handle that completes when the handle has been
                accepted or rejected
        """
        self.reset()
        if not self._cli_land.wait_for_server(timeout_sec=self._timeout_sec):
            self.get_logger().error("No action server available")
            return
        goal = Land.Goal()
        self.get_logger().info(f"Sending goal to `land`")
        future = self._cli_land.send_goal_async(goal)
        future.add_done_callback(functools.partial(self._action_response, "land"))
        return future

    #######################
    ## Feedback callbacks
    #######################
    def _feedback_arm_takeoff(self, feedback):
        self.get_logger().info(f"`arm_takeoff` feedback: {feedback.feedback.distance}m", throttle_duration_sec=2.0)
