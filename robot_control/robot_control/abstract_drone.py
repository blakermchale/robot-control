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
# ROS messages
from robot_control_interfaces.action import ArmTakeoff, Land


class ADrone(AVehicle):
    def __init__(self, log_level="info", instance=0):
        super().__init__(log_level=log_level, instance=instance)

        # Actions
        self._server_arm_takeoff = ActionServer(self, ArmTakeoff, "arm_takeoff", self._handle_arm_takeoff_goal,
            cancel_callback=self._handle_arm_takeoff_cancel)
        self._server_land = ActionServer(self, Land, "land", self._handle_land_goal,
            cancel_callback=self._handle_land_cancel)
        self.get_logger().info("ADrone initialized")

    def takeoff(self, alt: float):
        """Takes vehicle off from current location.

        Args:
            alt (float): altitude in meters

        Returns:
            bool: Flag indicating command was sent
        """
        raise NotImplementedError

    def land(self):
        """Lands vehicle at current location.
        
        Returns:
            bool: Flag indicating command was sent
        """
        raise NotImplementedError

    def is_landed(self):
        """Returns if vehicle has landed.

        Returns:
            bool: Whether the vehicle is landed
        """
        raise NotImplementedError

    #######################
    ## Actions
    #######################
    def _precheck_go_waypoint_goal(self):
        if not self.is_armed():
            self.get_logger().warn("GoWaypoint: aborted not armed")
            return False
        if self.is_landed():
            self.get_logger().warn("GoWaypoint: aborted not taken off")
            return False
        return True

    def _handle_arm_takeoff_goal(self, goal):
        """Action callback for arming and taking off vehicle.
        """
        self.get_logger().debug("ArmTakeoff: arming")
        self.arm()
        self.get_logger().debug(f"ArmTakeoff: sending {goal.request.altitude} m")
        if not self.takeoff(goal.request.altitude):
            self.get_logger().error("ArmTakeoff: could not send command")
            goal.abort()
            return ArmTakeoff.Result()
        self.get_logger().debug("ArmTakeoff: waiting to reach altitude")
        feedback_msg = ArmTakeoff.Feedback()
        start_time = self.get_clock().now()
        init_position = self._position
        while True:
            distance = self.distance_to_target()    
            reached = self.reached_target(distance=distance)        
            if self.get_clock().now() - start_time > self._wait_moved and not self.has_moved(init_position):
                self.get_logger().error("ArmTakeoff: hasn't moved aborting")
                self.disarm()
                goal.abort()
                return ArmTakeoff.Result()
            if distance is not None: #publish distance to target
                feedback_msg.distance = float(distance)
                goal.publish_feedback(feedback_msg)
            if goal.is_cancel_requested:
                goal.canceled() #handle cancel action
                return ArmTakeoff.Result()
            if reached:
                self.get_logger().info("ArmTakeoff: reached destination")
                self._success_arm_takeoff()
                goal.succeed() #handle sucess
                return ArmTakeoff.Result()
            self._check_rate.sleep()

    def _handle_arm_takeoff_cancel(self, cancel):
        """Callback for cancelling arm_takeoff action. Must be used with ActionServer.
        """
        self.land()
        self.get_logger().info(f'ArmTakeoff: cancelling...')
        while not self._check_success_land() and not self.is_landed():
            continue
        self.disarm()
        self.get_logger().info(f'ArmTakeoff: cancelled!')
        return CancelResponse.ACCEPT

    # FIXME: not necessary once landed state is fixed
    def _success_arm_takeoff(self):
        """Method that is ran after successful arm takeoff."""
        pass

    def _handle_land_goal(self, goal):
        """Action callback to land vehicle.
        """
        self.get_logger().debug("Land: sent command")
        if not self.land():
            self.get_logger().error("Land: could not send command")
            goal.abort()
            return Land.Result()
        self.get_logger().debug("Land: waiting")
        start_time = self.get_clock().now()
        init_position = self._position
        while True:
            if self.get_clock().now() - start_time > self._wait_moved and not self.has_moved(init_position):
                self.get_logger().error("Land: hasn't moved aborting")
                goal.abort()
                return Land.Result()
            if goal.is_cancel_requested:
                goal.canceled()
                return Land.Result()
            if self.is_landed():
                if not self.disarm(): self.get_logger().warn("Land: could not disarm at end")
                self.get_logger().info("Land: success!")
                goal.succeed()
                return Land.Result()
            self._check_rate.sleep()

    def _handle_land_cancel(self, cancel):
        """Callback for cancelling land action. Must be used with ActionServer."""
        self.get_logger().info('Land: cancelling...')
        self.halt()
        self.get_logger().info('Land: canceled!')
        return CancelResponse.ACCEPT
