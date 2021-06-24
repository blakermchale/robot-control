#!/usr/bin/env python
# This package
from robot_control.abstract_vehicle import AVehicle
# ROS packages
import rclpy
from rclpy.action import ActionServer, CancelResponse, ActionClient
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

from robot_control_interfaces.action import ArmTakeoff, Land


class ADrone(AVehicle):
    def __init__(self, log_level="info", instance=0):
        super().__init__(log_level=log_level, instance=instance)

        # Actions
        self._server_arm_takeoff = ActionServer(self, ArmTakeoff, "arm_takeoff", self._handle_arm_takeoff_goal,
            cancel_callback=self._handle_arm_takeoff_cancel)
        self._server_land = ActionServer(self, Land, "land", self._handle_land_goal,
            cancel_callback=self._handle_land_cancel)
        print("ADrone")

    def takeoff(self, alt: float):
        """Takes vehicle off from current location.

        Args:
            alt (float): altitude in meters
        """
        raise NotImplementedError

    def land(self):
        """Lands vehicle at current location."""
        raise NotImplementedError

    def is_landed(self):
        """Returns if vehicle has landed.

        Returns:
            bool: Whether the vehicle is landed
        """
        raise NotImplementedError

    # TODO: implement arm takeoff sequence with methods that aren't implemented
    def _handle_arm_takeoff_goal(self, goal):
        """Action callback for arming and taking off vehicle.
        """
        self.arm()
        self.takeoff(goal.request.altitude)
        feedback_msg = ArmTakeoff.Feedback()
        while True:
            self._check_rate.sleep()
            distance = self.distance_to_target()    
            reached = self.reached_target(distance=distance)        
            if distance is not None: #publish distance to target
                feedback_msg.distance = float(distance)
                goal.publish_feedback(feedback_msg)
            if goal.is_cancel_requested:
                goal.canceled() #handle cancel action
                return ArmTakeoff.Result()
            if reached:
                self.get_logger().info("ArmTakeoff: reached destination")
                goal.succeed() #handle sucess
                return ArmTakeoff.Result()


    def _handle_arm_takeoff_cancel(self, cancel):
        """Callback for cancelling arm_takeoff action. Must be used with ActionServer.
        """
        self.land()
        self.get_logger().info(f'ArmTakeoff: cancelling takeoff...')
        while not self.is_landed():
            continue
        self.get_logger().info(f'ArmTakeoff: action cancelled!')
        return CancelResponse.ACCEPT

    def _handle_land_goal(self, goal):
        """Action callback to land vehicle.
        """        
        self.land()
        while True:
            self._check_rate.sleep()
            if goal.is_cancel_requested:
                goal.canceled()
                return Land.Result()
            if self.is_landed():
                goal.succeed()
                return Land.Result()

    def _handle_land_cancel(self, cancel):
        """Callback for cancelling land action. Must be used with ActionServer.
        """
        self.get_logger().info('Land: cancelling...')
        self.halt()
        self.get_logger().info('Land: canceled!')
        return CancelResponse.ACCEPT
