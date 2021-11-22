#!/usr/bin/env python3
from numpy.lib.financial import _rate_dispatcher
from ..abstract_vehicle import AVehicle
from ..cli.common import get_parameter_value_msg_from_type
from ..utils.structs import Frame

from mavros_msgs.srv import CommandBool, CommandLong, ParamSetV2
from mavros_msgs.msg import State, StatusText, Altitude, PositionTarget
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header, String
from rcl_interfaces.srv import SetParameters, ListParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue

from rclpy.duration import Duration
from rclpy.qos import QoSPresetProfiles
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor

from enum import IntEnum, auto
import threading

# Mode's are custom
# https://discuss.px4.io/t/how-to-switch-to-offboard-mode-through-mavlink/9812
# https://github.com/PX4/PX4-Autopilot/blob/master/src/modules/commander/px4_custom_mode.h#L45-L55
class PX4Mode(IntEnum):
    MANUAL=1
    ALTCTL=auto()
    POSCTL=auto()
    AUTO=auto()
    ACRO=auto()
    OFFBOARD=auto()
    STABILIZED=auto()
    RATTITUDE=auto()
    # sub mode auto
    READY=auto()
    TAKEOFF=auto()
    LOITER=auto()
    MISSION=auto()
    RTL=auto()
    LAND=auto()
    RTGS=auto()


class PX4Command(IntEnum):
    MAV_CMD_DO_SET_MODE=176


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
        self._global_lla = None
        self._amsl_alt = None

        # Services
        self._cli_arm = self.create_client(CommandBool, "mavros/cmd/arming")
        self._cli_command = self.create_client(CommandLong, "mavros/cmd/command")
        # self._cli_set_param = self.create_client(ParamSetV2, "mavros/param/set")
        # Subscribers
        self._sub_state = self.create_subscription(State, "mavros/state", self._cb_state, 10)
        self._sub_local_odom = self.create_subscription(Odometry, "mavros/local_position/odom", self._cb_local_odom, QoSPresetProfiles.SENSOR_DATA.value)
        self._sub_global_lla = self.create_subscription(NavSatFix, "mavros/global_position/global", self._cb_global_lla, QoSPresetProfiles.SENSOR_DATA.value)
        self._sub_statustext = self.create_subscription(StatusText, "mavros/statustext/recv", self._cb_statustext, QoSPresetProfiles.SENSOR_DATA.value)
        self._sub_altitude = self.create_subscription(Altitude, "mavros/altitude", self._cb_altitude, QoSPresetProfiles.SENSOR_DATA.value)
        # self._sub_pose = self.create_subscription(PoseStamped, "mavros/local_position/pose", self._cb_pose, 10)
        # Publishers
        self._pub_setpoint_local = self.create_publisher(PoseStamped, "mavros/setpoint_position/local", 10)
        self._pub_setpoint_local_raw = self.create_publisher(PositionTarget, "mavros/setpoint_raw/local", 10)
        self._pub_px4_mode = self.create_publisher(String, "mode", 10)

        # Set default parameters for sim
        threading.Thread(target=self._send_sim_params,daemon=True).start()
        
        self.get_logger().debug("Initialized MAVROS Vehicle!")

    def update(self):
        super().update()
        self._pub_px4_mode.publish(String(data=self._state.mode.name))

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        target_pose = PoseStamped()
        target_pose.header = header
        target_pose.pose = self.target.get_msg()
        target_pose.pose.position.z *= -1
        self._pub_setpoint_local.publish(target_pose)
        # pos_target = PositionTarget()
        # pos_target.position = self.target.position.get_point_msg()
        # pos_target.yaw = self.target.euler.z
        # pos_target.header = header
        # pos_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        # # pos_target.velocity = self.target.  # TODO: need to add velocities to target, make it odom
        # self._pub_setpoint_local_raw.publish(pos_target)

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

    def send_waypoint(self, x: float, y: float, z: float, heading: float, frame: int = Frame.LOCAL_NED):
        if frame == Frame.LOCAL_NED:
            self.set_target(x, y, z, heading)
        else:
            self.get_logger().error(f"Frame {frame.name} not implemented yet")
            return False
        while not self.in_offboard():
            if not self.set_offboard():
                self.get_logger().error("Couldn't set offboard mode", throttle_duration_sec=2.0)
                # return False
            else:
                self.get_logger().info("Set offboard", throttle_duration_sec=2.0)
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
        return self.set_mode(PX4Mode.OFFBOARD)

    def set_mode(self, mode: PX4Mode):
        """Sets PX4 mode using the associations from https://discuss.px4.io/t/mav-cmd-do-set-mode-all-possible-modes/8495/2.

        Here's an example of sending this command from the shell:
        ros2 service call /drone_0/mavros/cmd/command mavros_msgs/srv/CommandLong "{'command':176,'param1':217,'param2':1}"

        Args:
            mode (PX4Mode): Mode for PX4 to switch to.

        Returns:
            bool: Flag indicating if mode was set.
        """
        # base_mode = 219
        # main_mode = 1
        # sub_mode = 0
        # if mode == PX4Mode.MANUAL:
        #     base_mode = 217
        #     main_mode = 1
        # elif mode == PX4Mode.STABILIZED:
        #     base_mode = 209
        #     main_mode = 7
        # elif mode == PX4Mode.ACRO:
        #     base_mode = 209
        #     main_mode = 5
        # elif mode == PX4Mode.RATTITUDE:
        #     base_mode = 193
        #     main_mode = 8
        # elif mode == PX4Mode.ALTCTL:
        #     # base_mode = 193
        #     base_mode = 209
        #     main_mode = 2
        # elif mode == PX4Mode.OFFBOARD:
        #     base_mode = 209
        #     main_mode = 6
        # elif mode == PX4Mode.POSCTL:
        #     base_mode = 209
        #     main_mode = 3
        req = CommandLong.Request()
        req.command = int(PX4Command.MAV_CMD_DO_SET_MODE) #MAV_CMD_DO_SET_MODE
        req.param1 = float(209)  # custom
        req.param2 = float(mode.value)
        resp = self._cli_command.call(req)
        if not resp.success:
            self.get_logger().error(f"Received ACK result from set mode: {resp.result}")
            return False
        return True

    ################
    ## PX4 Checks ##
    ################
    def in_offboard(self):
        return self._state.mode == PX4Mode.OFFBOARD

    #################
    ## Subscribers ##
    #################
    def _cb_state(self, msg: State):
        self._state.from_msg(msg)

    def _cb_local_odom(self, msg: Odometry):
        self.pose.set_pose(msg.pose.pose)
        self.lin_vel.v3 = msg.twist.twist.linear
        self.ang_vel.v3 = msg.twist.twist.angular
        # convert to NED
        self.position.y *= -1
        self.position.z *= -1

    def _cb_global_lla(self, msg: NavSatFix):
        self._global_lla = msg

    def _cb_statustext(self, msg: StatusText):
        self.get_logger().info(f"PX4 Log: {msg.text}")

    def _cb_altitude(self, msg: Altitude):
        self._amsl_alt = msg.amsl

    #############
    ## Helpers ##
    #############
    def _send_sim_params(self):
        exe = MultiThreadedExecutor()
        exe.add_node(self)
        self._cli_list_params = self.create_client(ListParameters, "mavros/param/list_parameters")
        req = ListParameters.Request()
        while True:
            future = self._cli_list_params.call_async(req)
            exe.spin_until_future_complete(future)
            if "NAV_RCL_ACT"  in future.result().result.names:
                self.get_logger().debug("Set sim params")
                break
            self.get_logger().warn(f"Couldn't find sim params in list", throttle_duration_sec=3.0)
        self._cli_set_params = self.create_client(SetParameters, "mavros/param/set_parameters")
        req = SetParameters.Request()
        NAV_RCL_ACT = ParameterMsg(name="NAV_RCL_ACT",value=ParameterValue(type=Parameter.Type.INTEGER.value,integer_value=0))
        req.parameters = [NAV_RCL_ACT]
        future = self._cli_set_params.call_async(req)
        exe.spin_until_future_complete(future)

    ################
    ## Properties ##
    ################
