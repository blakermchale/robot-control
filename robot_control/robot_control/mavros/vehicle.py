#!/usr/bin/env python3

from rclpy.duration import Duration
from robot_control.abstract_vehicle import AVehicle
from mavros_msgs.srv import CommandBool, CommandLong, ParamSetV2
from mavros_msgs.msg import State, StatusText
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from rclpy.qos import QoSPresetProfiles

from enum import IntEnum

class PX4Mode(IntEnum):
    MANUAL=0
    ACRO=1
    ALTCTL=2
    POSCTL=3
    OFFBOARD=4
    STABILIZED=5
    RATTITUDE=6
    MISSION=7
    LOITER=8
    RTL=9
    LAND=10
    RTGS=11
    READY=12
    TAKEOFF=13


class PX4State:
    def __init__(self):
        self._mode = PX4Mode.LOITER
        self.armed = False

    def from_msg(self, msg: State):
        val = msg.mode
        if val.startswith("AUTO."):
            val = val.replace("AUTO.","")
        self.mode = PX4Mode[val]
        self.armed = msg.armed


class Vehicle(AVehicle):
    def __init__(self, instance=0):
        super().__init__(instance=instance)

        # Overrides
        self._wait_moved = Duration(seconds=20)
        
        # Initial vars
        self._state = PX4State()
        self._global_lla= None

        # Services
        self._cli_arm = self.create_client(CommandBool, "mavros/cmd/arming")
        self._cli_command = self.create_client(CommandLong, "mavros/cmd/command")
        # self._cli_set_param = self.create_client(ParamSetV2, "mavros/param/set")
        # Subscribers
        self._sub_state = self.create_subscription(State, "mavros/state", self._cb_state, 10)
        self._sub_local_odom = self.create_subscription(Odometry, "mavros/local_position/odom", self._cb_local_odom, QoSPresetProfiles.SENSOR_DATA.value)
        self._sub_global_lla = self.create_subscription(NavSatFix, "mavros/global_position/global", self._cb_global_lla, QoSPresetProfiles.SENSOR_DATA.value)
        self._sub_statustext = self.create_subscription(StatusText, "mavros/statustext/recv", self._cb_statustext, QoSPresetProfiles.SENSOR_DATA.value)
        # self._sub_pose = self.create_subscription(PoseStamped, "mavros/local_position/pose", self._cb_pose, 10)
        # Publishers
        self._pub_setpoint_local = self.create_publisher(Odometry, "mavros/setpoint_position/odom", 10)

        # req = ParamSetV2.Request()
        # req.param_id = 
        # self._cli_set_param
        
        self.get_logger().debug("Initialized MAVROS Vehicle!")

    ######################
    ## Control commands ##
    ######################
    def arm(self):
        req = CommandBool.Request()
        req.value = True
        resp = self._cli_arm.call(req)
        if not resp.success:
            self.get_logger().error(f"Failed to arm (MAV_RESULT={resp.result})")
            return False
        return True

    def disarm(self):
        req = CommandBool.Request()
        req.value = False
        resp = self._cli_arm.call(req)
        if not resp.success:
            self.get_logger().error(f"Failed to disarm (MAV_RESULT={resp.result})")
            return False
        return True

    #####################
    ## Checking states ##
    #####################
    def is_armed(self):
        return self._state.armed

    ##################
    ## PX4 commands ##
    ##################
    def set_offboard(self):
        """Sets the drone to `OFFBOARD` mode.
        """
        req = CommandLong.Request()
        req.command = int(176) #MAV_CMD_DO_SET_MODE
        req.param1 = float(209.0) #OFFBOARD
        req.param2 = float(6.0)
        self._cli_command.call(req)

    #################
    ## Subscribers ##
    #################
    def _cb_state(self, msg: State):
        self._state.from_msg(msg)

    def _cb_local_odom(self, msg: Odometry):
        self.pose.set_pose(msg.pose.pose)
        self.lin_vel.v3 = msg.twist.twist.linear
        self.ang_vel.v3 = msg.twist.twist.angular

    def _cb_global_lla(self, msg: NavSatFix):
        self._global_lla = msg

    def _cb_statustext(self, msg: StatusText):
        self.get_logger().info(f"PX4 Log: {msg.text}")

    ################
    ## Properties ##
    ################
