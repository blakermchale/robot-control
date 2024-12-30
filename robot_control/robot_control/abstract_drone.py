#!/usr/bin/env python
# This package
from robot_control.abstract_vehicle import AVehicle
# ROS packages
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from rclpy.action.server import ServerGoalHandle
# ROS messages
from robot_control_interfaces.action import ArmTakeoff, Land
import numpy as np


class ADrone(AVehicle):
    def __init__(self, instance=0):
        super().__init__(instance=instance)

        # Actions
        self._server_arm_takeoff = ActionServer(self, ArmTakeoff, "arm_takeoff", execute_callback=self._handle_arm_takeoff_execute,
            cancel_callback=self._handle_arm_takeoff_cancel, goal_callback=self._handle_arm_takeoff_goal)
        self._server_land = ActionServer(self, Land, "land", execute_callback=self._handle_land_execute,
            cancel_callback=self._handle_land_cancel, goal_callback=self._handle_land_goal)

        self.get_logger().debug("ADrone initialized")

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

    def _handle_arm_takeoff_goal(self, goal_request):
        """Action callback for accepting goal request.
        """
        if self._active_action_server:
            self.get_logger().info("ArmTakeoff: Request already active. Stop other request to submit new one. Rejecting.")
            return GoalResponse.REJECT
        self._active_action_server = True
        return GoalResponse.ACCEPT

    def _handle_arm_takeoff_execute(self, goal: ServerGoalHandle):
        """Action callback for arming and taking off vehicle.
        """
        self.get_logger().debug("ArmTakeoff: arming")
        self.arm()
        self.get_logger().debug(f"ArmTakeoff: sending {goal.request.altitude} m")
        if not self.takeoff(goal.request.altitude):
            self.get_logger().error("ArmTakeoff: could not send command")
            goal.abort()
            self._abort_arm_takeoff()
            return ArmTakeoff.Result()
        self.get_logger().debug("ArmTakeoff: waiting to reach altitude")
        feedback_msg = ArmTakeoff.Feedback()
        start_time = self.get_clock().now()
        init_position = self.position.copy()
        while True:
            distance = self.distance_to_target()    
            reached = self.reached_target(distance=distance)
            if self.get_clock().now() - start_time > self._wait_moved and not self.has_moved(init_position):
                self.get_logger().error("ArmTakeoff: hasn't moved aborting")
                self.disarm()
                self._abort_arm_takeoff()
                goal.abort()
                self._active_action_server = False
                return ArmTakeoff.Result()
            if distance is not None: #publish distance to target
                feedback_msg.distance = float(distance)
                goal.publish_feedback(feedback_msg)
            if goal.is_cancel_requested:
                # TODO(bmchale): is it safer to land when cancelling or halt?
                # self.land()
                self.halt()
                self.get_logger().info(f'ArmTakeoff: cancelling...')
                # while not self.is_landed():
                #     continue
                # self.disarm()
                self.get_logger().info(f'ArmTakeoff: cancelled!')
                goal.canceled() #handle cancel action
                self._abort_arm_takeoff()
                self._active_action_server = False
                return ArmTakeoff.Result()
            if reached:
                self.get_logger().info("ArmTakeoff: reached destination")
                self._success_arm_takeoff()
                goal.succeed() #handle sucess
                self._active_action_server = False
                return ArmTakeoff.Result()
            self._check_rate.sleep()

    def _handle_arm_takeoff_cancel(self, cancel):
        """Callback for cancelling arm_takeoff action. Must be used with ActionServer.
        """
        return CancelResponse.ACCEPT

    # FIXME: not necessary once landed state is fixed
    def _success_arm_takeoff(self):
        """Method that is ran after successful arm takeoff."""
        pass

    def _abort_arm_takeoff(self):
        """Method that is ran after failed arm takeoff."""
        pass

    def _handle_land_goal(self, goal_request):
        """Action callback for accepting goal request.
        """
        if self._active_action_server:
            self.get_logger().info("Land: Request already active. Stop other request to submit new one. Rejecting.")
            return GoalResponse.REJECT
        self._active_action_server = True
        return GoalResponse.ACCEPT

    def _handle_land_execute(self, goal: ServerGoalHandle):
        """Action callback to land vehicle.
        """
        self.get_logger().debug("Land: sent command")
        if not self.land():
            self.get_logger().error("Land: could not send command")
            goal.abort()
            self._active_action_server = False
            return Land.Result()
        self.get_logger().debug("Land: waiting")
        start_time = self.get_clock().now()
        init_position = self.position.copy()
        while True:
            if self.get_clock().now() - start_time > self._wait_moved and not self.has_moved(init_position):
                self.get_logger().error("Land: hasn't moved aborting")
                goal.abort()
                self._active_action_server = False
                return Land.Result()
            if goal.is_cancel_requested:
                self.get_logger().info('Land: cancelling...')
                self.halt()
                self.get_logger().info('Land: canceled!')
                goal.canceled()
                self._active_action_server = False
                return Land.Result()
            if self.is_landed():
                if not self.disarm(): self.get_logger().warn("Land: could not disarm at end")
                self.get_logger().info("Land: success!")
                goal.succeed()
                self._active_action_server = False
                return Land.Result()
            self._check_rate.sleep()

    def _handle_land_cancel(self, cancel):
        """Callback for cancelling land action. Must be used with ActionServer."""
        return CancelResponse.ACCEPT
