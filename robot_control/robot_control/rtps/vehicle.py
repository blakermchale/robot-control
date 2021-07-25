#!/usr/bin/env python3

from robot_control.abstract_vehicle import AVehicle

from px4_msgs.msg import VehicleStatus, VehicleOdometry, Timesync, VehicleLocalPosition, TrajectorySetpoint, VehicleCommand, OffboardControlMode

import numpy as np


class Vehicle(AVehicle):
    def __init__(self, instance=0):
        super().__init__(instance=instance)

        # Internal variables
        self._mode = VehicleStatus()
        # Timestamp for published RTPS messages, must match incoming time
        self._rtps_timestamp = 0
        # Main trajectory setpoint that needs to be published constantly
        self._trajectory_setpoint = TrajectorySetpoint()
        # Offboard params that needs to be published constantly
        self._offboard_mode = OffboardControlMode()
        self._offboard_mode.position = True
        self._offboard_mode.velocity = False
        self._offboard_mode.acceleration = False
        self._offboard_mode.attitude = False
        self._offboard_mode.body_rate = False

        #Wait for a vehicle connection
        while self.count_publishers("RTPS/Timesync_PubSubTopic") <= 0:
            self.get_logger().info(f"Waiting for vehicle with name '{self._namespace}'...", throttle_duration_sec=5.0)

        # FIXME: don't only use default rtps topics, enable other ones like land detected
        # uOrb Publishers
        self._pub_trajectory_setpoint = self.create_publisher(TrajectorySetpoint, "RTPS/TrajectorySetpoint_PubSubTopic", 1)
        self._pub_vehicle_command = self.create_publisher(VehicleCommand, "RTPS/VehicleCommand_PubSubTopic", 1)
        self._pub_offboard_mode = self.create_publisher(OffboardControlMode, "RTPS/OffboardControlMode_PubSubTopic", 1)

        # uOrb Subscribers
        self._sub_mode = self.create_subscription(VehicleStatus, "RTPS/VehicleStatus_PubSubTopic", self._callback_mode, 1)
        # self._sub_home = self.create_subscription(HomePosition, "RTPS/HomePosition_PubSubTopic", self._callback_home_coordinate, 1)
        self._sub_odom = self.create_subscription(VehicleOdometry, "RTPS/VehicleOdometry_PubSubTopic", self._callback_odometry, 1)
        self._sub_timesync = self.create_subscription(Timesync, "RTPS/Timesync_PubSubTopic", self._callback_rtps_timestamp, 1)
        # self._sub_vehicle_land = self.create_subscription(VehicleLandDetected, "RTPS/VehicleLandDetected_PubSubTopic", self._callback_vehicle_land, 1)
        # self._sub_global_position = self.create_subscription(VehicleGlobalPosition, "RTPS/VehicleGlobalPosition_PubSubTopic", self._callback_global_position, 1)
        # self._sub_vehicle_command_ack = self.create_subscription(VehicleCommandAck, "RTPS/VehicleCommandAck_PubSubTopic", self._callback_vehicle_command_ack, 1)
        self._sub_local_position = self.create_subscription(VehicleLocalPosition, "RTPS/VehicleLocalPosition_PubSubTopic", self._callback_local_position, 1)

        self.get_logger().debug("Vehicle initialized")

    def update(self):
        # Only publish setpoints when in offboard, publishing otherwise affect takeoff and land
        if self._changing_offboard:
            self._publish_setpoint()
            self._publish_offboard_mode()
            if self._setpoint_count < self._max_setpoints:
                self._setpoint_count += 1

        # Disable publishing setpoints when not in offboard
        if not self.in_offboard() and (self.get_clock().now() - self._prev_reset_time) > self._reset_wait_time and self._setpoint_count == self._max_setpoints:
            self._reset_offboard_changes(msg=f"Resetting offboard changes!")
        super().update()

    #######################
    ## Control commands
    #######################
    def arm(self):
        if not (self._mode.arming_state == VehicleStatus.ARMING_STATE_INIT or self._mode.arming_state == VehicleStatus.ARMING_STATE_STANDBY or self._mode.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            self.get_logger().error(f'Vehicle arming state is {self._mode.arming_state} (see VehicleStatus msg)')
            return False
        elif self._mode.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().debug("Vehicle already armed!")
            return True

        self.send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        return True

    def disarm(self):
        # TODO: add check to see if vehicle is already disarmed and don't send command in that case
        if not self._vehicle_landed.landed:
            self.get_logger().error(f'Vehicle is not landed, cannot disarm')
            return False
        self.send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        return True

    def halt(self):
        self.get_logger().debug("Halting!")
        self._reset_offboard_changes()
        # TODO: is this the behavior we want? when switching to hold it tends to backtrack to its position when it was switched which is problematic when the vehicle is moving fast since its momentum carries it a far distance from that point
        self.set_mode(PX4Mode.HOLD)

    def send_waypoint(self, x: float, y: float, z: float, heading: float, frame: int):
        raise NotImplementedError

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float, frame: int = Frame.LOCAL_NED):
        raise NotImplementedError

    ####################
    ## Checking states
    ####################
    def is_armed(self):
        return self._mode.arming_state == VehicleStatus.ARMING_STATE_ARMED

    #######################
    ## Helpers
    #######################
    def send_vehicle_cmd(self, command, param1:float=np.nan, param2:float=np.nan, 
                         param3:float=np.nan, param4=np.nan, param5=np.nan, param6=np.nan, 
                         param7=np.nan, wait=True) -> bool:
        """Send vehicle command over RTPS.

        Args:
            command (int): Command ID, as defined MAVLink by uint16 VEHICLE_CMD enum.
            param1 (float): Parameter 1, as defined by MAVLink uint16 VEHICLE_CMD enum.
            param2 (float, optional): Parameter 2, as defined by MAVLink uint16 VEHICLE_CMD enum. Defaults to None.
            param3 (float, optional): Parameter 3, as defined by MAVLink uint16 VEHICLE_CMD enum. Defaults to None.
            param4 (float, optional): Parameter 4, as defined by MAVLink uint16 VEHICLE_CMD enum. Defaults to None.
            param5 (float, optional): Parameter 5, as defined by MAVLink uint16 VEHICLE_CMD enum. Defaults to None.
            param6 (float, optional): Parameter 6, as defined by MAVLink uint16 VEHICLE_CMD enum. Defaults to None.
            param7 (float, optional): Parameter 7, as defined by MAVLink uint16 VEHICLE_CMD enum. Defaults to None.
            wait (bool, optional): Flag indicating if this function should wait for an ACK. Defaults to True.

        Returns:
            bool: Whether the command was sent and received successfully (if waiting)
        """        
        msg = VehicleCommand()
        msg.timestamp = self._rtps_timestamp
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.command = int(command)
        msg.target_system = self.instance + 1
        msg.target_component = 1
        msg.source_system = self.instance + 1
        msg.source_component = 1
        msg.from_external = True
        self._pub_vehicle_command.publish(msg)
        if not wait:
            return True

        wait_dur = 0.5 * 10**6 # seconds
        max_sends = 20  # maximum times to send command before returning false
        sends = 0
        curr_time = self._rtps_timestamp
        stale_times = 0
        max_stale_times = 10
        rate = self.create_rate(100)
        while True:
            rate.sleep()
            self.get_logger().info(f"Waiting for command {msg.command} to be acknowleged...", throttle_duration_sec=5.0)
            command = self._vehicle_command_ack.command
            target_system = self._vehicle_command_ack.target_system
            target_component = self._vehicle_command_ack.target_component
            timestamp = self._vehicle_command_ack.timestamp
            result = self._vehicle_command_ack.result
            # Count occurrences where time has not changed in this loop
            stale_times = stale_times + 1 if curr_time == self._rtps_timestamp else 0
            if stale_times >= 100:
                self.get_logger().error("Time is not being updated, loop lockup occurring")
                return False
            curr_time = self._rtps_timestamp
            # self.get_logger().debug(f"sent cmd: {msg.command}, sent time: {msg.timestamp}, cmd: {command}, tgs: {target_system}, tgc: {target_component}, time: {timestamp}, result: {result}", throttle_duration_sec=1.0)
            # Attempt to send command again after a period of time has passed
            if curr_time - msg.timestamp > wait_dur and sends < max_sends:
                sends += 1
                self.get_logger().debug(f"Republishing command {msg.command} since no response has been received! Sent {sends} times!")
                msg.timestamp = self._rtps_timestamp
                self._pub_vehicle_command.publish(msg)
                continue
            elif sends >= max_sends:
                self.get_logger().error(f"Tried to send command {max_sends} times but never received ack!")
                return False
            same_system = (msg.source_system == target_system and msg.source_component == target_component and msg.command == command and msg.timestamp < timestamp)
            if not same_system:
                self.get_logger().debug(f"Ack message is not from this request... from system {target_system} and component {target_component} at time {timestamp} with command {command}", throttle_duration_sec=5.0)
                continue
            if result == VehicleCommandAck.VEHICLE_RESULT_ACCEPTED:
                return True
            elif result == VehicleCommandAck.VEHICLE_RESULT_TEMPORARILY_REJECTED:
                self.get_logger().error("Command temporarily rejected...")
                return False
            elif result == VehicleCommandAck.VEHICLE_RESULT_DENIED:
                self.get_logger().error("Command denied!")
                return False
            elif result == VehicleCommandAck.VEHICLE_RESULT_UNSUPPORTED:
                self.get_logger().error("Command not supported!")
                return False
            elif result == VehicleCommandAck.VEHICLE_RESULT_FAILED:
                self.get_logger().error("Command failed!")
                return False
            elif result == VehicleCommandAck.VEHICLE_RESULT_IN_PROGRESS:
                self.get_logger().debug("Command in progress...")
                continue

    #######################
    ## Private helpers
    #######################
    def _set_offboard_mode(self, position:bool=False, velocity:bool=False, acceleration:bool=False, attitude:bool=False, body_rate:bool=False):
        """Set Offboard mode parameters.
        """  
        self._offboard_mode = OffboardControlMode()      
        self._offboard_mode.position = position
        self._offboard_mode.velocity = velocity
        self._offboard_mode.acceleration = acceleration
        self._offboard_mode.attitude = attitude
        self._offboard_mode.body_rate = body_rate

    def _publish_offboard_mode(self):
        """Publsih Offboard mode to ROS2.
        """        
        self._offboard_mode.timestamp = self._rtps_timestamp
        self._pub_offboard_mode.publish(self._offboard_mode)

    #################################
    ## Subscriber callbacks
    #################################
    def _callback_mode(self, msg: VehicleStatus):
        self._mode = msg

    def _callback_local_position(self, msg: VehicleLocalPosition):
        if not (msg.xy_valid and msg.xy_global and msg.z_global):
            self.get_logger().warn("XYZ position or global reference is not valid...", throttle_duration_sec=5.0)
            return
        self._position = np.asfarray([msg.x, msg.y, msg.z])

    def _callback_odometry(self, msg: VehicleOdometry):
        pass

    def _callback_rtps_timestamp(self, msg: Timesync):
        self._rtps_timestamp = msg.timestamp
