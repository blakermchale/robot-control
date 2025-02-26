#!/usr/bin/env python3
'''
Drone Client
=======================

Generic drone client to ROS2 actions and topics in companion_computing. Easy publishing, calling,
and sending goals.
'''
from rclpy.action import ActionClient

from robot_control_interfaces.action import ArmTakeoff, Land

from .vehicle_client import VehicleClient
from ros2_utils.cli import setup_send_action
from rclpy.task import Future


class DroneClient(VehicleClient):
    def __init__(self, executor, namespace=None, log_feedback=True, add_to_executor=True):
        super().__init__(executor, namespace=namespace, log_feedback=log_feedback, add_to_executor=add_to_executor)
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
        @setup_send_action(self, self._cli_arm_takeoff, self._feedback_arm_takeoff)
        def send_action():
            return ArmTakeoff.Goal(altitude=altitude)
        return send_action

    def send_land(self) -> Future:
        """Sends land command to drone.

        Returns:
            Future: a Future instance to a goal handle that completes when the handle has been
                accepted or rejected
        """
        @setup_send_action(self, self._cli_land, None)
        def send_action():
            return Land.Goal()
        return send_action
    
    #######################
    ## Feedback callbacks
    #######################
    def _feedback_arm_takeoff(self, feedback):
        self.feedback = feedback.feedback
        self.get_logger().info(f"`arm_takeoff` feedback: {feedback.feedback.distance}m", throttle_duration_sec=2.0)


def create_drone_client(executor, namespace=None, log_feedback=True, add_to_executor=False) -> DroneClient:
    client = DroneClient(executor, namespace, log_feedback=log_feedback, add_to_executor=add_to_executor)
    executor.add_node(client)
    return client
