#!/usr/bin/env python3
from ros2_utils import convert_axes_from_msg, AxesFrame
from ..abstract_vehicle import AVehicle
from ..utils.structs import Frame

from mavros_msgs.srv import CommandBool, ParamSetV2, SetMode
from mavros_msgs.msg import State, StatusText, Altitude, PositionTarget
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
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
        self.mode = PX4Mode.LOITER
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
        self._cli_set_mode = self.create_client(SetMode, "mavros/set_mode")
        # self._cli_set_param = self.create_client(ParamSetV2, "mavros/param/set")
        # Subscribers
        self._sub_state = self.create_subscription(State, "mavros/state", self._cb_state, 10)
        self._sub_local_odom = self.create_subscription(Odometry, "mavros/local_position/odom", self._cb_local_odom, QoSPresetProfiles.SENSOR_DATA.value)
        self._sub_global_lla = self.create_subscription(NavSatFix, "mavros/global_position/global", self._cb_global_lla, QoSPresetProfiles.SENSOR_DATA.value)
        self._sub_statustext = self.create_subscription(StatusText, "mavros/statustext/recv", self._cb_statustext, QoSPresetProfiles.SENSOR_DATA.value)
        self._sub_altitude = self.create_subscription(Altitude, "mavros/altitude", self._cb_altitude, QoSPresetProfiles.SENSOR_DATA.value)
        # self._sub_pose = self.create_subscription(PoseStamped, "mavros/local_position/pose", self._cb_pose, 10)
        # Publishers
        self._pub_setpoint_local_raw = self.create_publisher(PositionTarget, "mavros/setpoint_raw/local", 1)
        self._pub_setpoint_vel_local = self.create_publisher(Twist, "mavros/setpoint_velocity/cmd_vel_unstamped", 1)
        self._pub_px4_mode = self.create_publisher(String, "mode", 10)

        # Set default parameters for sim
        threading.Thread(target=self._send_sim_params,daemon=True).start()
        
        self.get_logger().debug("Initialized MAVROS Vehicle!")

    def update(self):
        super().update()
        self._pub_px4_mode.publish(String(data=self._state.mode.name))

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        if self.target_odom.twist.is_zero():
            pos_target = PositionTarget()
            pos_target.position = self.target.position.get_point_msg()
            pos_target.position.z *= -1
            pos_target.position.y *= -1
            pos_target.yaw = self.target.euler.z*-1
            pos_target.header = header
            pos_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            # pos_target.velocity = self.target_odom.lin_vel.get_vector3_msg()
            # pos_target.yaw_rate = self.target_odom.ang_vel.z
            # pos_target.velocity = self.target.  # TODO: need to add velocities to target, make it odom. tried but couldn't get it working
            self._pub_setpoint_local_raw.publish(pos_target)
        else:
            # reset target odom velocities after last velocity command has gotten old
            if self._old_time_velocity():
                self.target_odom.lin_vel = [0.0,0.0,0.0]
                self.target_odom.ang_vel = [0.0,0.0,0.0]
            else:
                self.target = self.pose.copy()
                twist_msg = self.target_odom.twist.get_msg()
                vx,vy,vz,_,_,yaw_rate = self.convert_velocity_frame(twist_msg.linear.x,twist_msg.linear.y,twist_msg.linear.z,
                    twist_msg.angular.x,twist_msg.angular.y,twist_msg.angular.z,Frame.FRD,Frame.LOCAL_NED)
                twist_msg.linear.x = vx
                twist_msg.linear.y = vy
                twist_msg.linear.z = vz
                twist_msg.angular.z = yaw_rate
                twist_msg = convert_axes_from_msg(twist_msg, AxesFrame.URHAND, AxesFrame.RHAND)
                self._pub_setpoint_vel_local.publish(twist_msg)

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

    def send_waypoint(self, x: float, y: float, z: float, heading: float, frame: Frame = Frame.LOCAL_NED):
        try:
            x, y, z, roll, pitch, heading = self.convert_position_frame(x, y, z, 0, 0, heading, frame, Frame.LOCAL_NED)
        except Exception as e:
            self.get_logger().error(str(e))
            return False
        self.set_target(x, y, z, heading)
        while not self.in_offboard():
            if not self.set_offboard():
                self.get_logger().error("Couldn't set offboard mode", throttle_duration_sec=2.0)
                # return False
            else:
                self.get_logger().info("Set offboard", throttle_duration_sec=2.0)
        return True

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float, frame: Frame = Frame.FRD):
        vx, vy, vz, roll_rate, pitch_rate, yaw_rate = self.convert_velocity_frame(vx, vy, vz, 0, 0, yaw_rate, frame, Frame.FRD)
        self.target_odom.lin_vel = [vx, vy, vz]
        self.target_odom.ang_vel = [roll_rate, pitch_rate, yaw_rate]
        while not self.in_offboard():
            if not self.set_offboard():
                self.get_logger().error("Couldn't set offboard mode", throttle_duration_sec=2.0)
                # return False
            else:
                self.get_logger().info("Set offboard", throttle_duration_sec=2.0)
        return True

    def halt(self):
        # TODO: don't perform as hard of a stop and update waypoint as vehicle slows down so it doesn't backtrack
        return self.send_waypoint(self.position.x, self.position.y, self.position.z, self.euler.z)

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

        Args:
            mode (PX4Mode): Mode for PX4 to switch to.

        Returns:
            bool: Flag indicating if mode was set.
        """
        req = SetMode.Request()
        req.custom_mode = mode.name if mode < PX4Mode.READY else f"AUTO.{mode.name}"
        resp = self._cli_set_mode.call(req)
        if not resp.mode_sent:
            self.get_logger().error(f"'{req.custom_mode}' not sent")
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
        self.odom.set_msg(convert_axes_from_msg(msg, AxesFrame.RHAND, AxesFrame.URHAND))

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
        cli_list_params = self.create_client(ListParameters, "mavros/param/list_parameters")
        req = ListParameters.Request()
        sim_names = set(["NAV_RCL_ACT","COM_RCL_EXCEPT"])
        while True:
            future = cli_list_params.call_async(req)
            exe.spin_until_future_complete(future)
            names = set(future.result().result.names)
            if sim_names.issubset(names):
                self.get_logger().debug("Set sim params")
                break
            self.get_logger().warn(f"Couldn't find sim params in list", throttle_duration_sec=3.0)
        cli_set_params = self.create_client(SetParameters, "mavros/param/set_parameters")
        req = SetParameters.Request()
        NAV_RCL_ACT = ParameterMsg(name="NAV_RCL_ACT",value=ParameterValue(type=Parameter.Type.INTEGER.value,integer_value=0))
        COM_RCL_EXCEPT = ParameterMsg(name="COM_RCL_EXCEPT",value=ParameterValue(type=Parameter.Type.INTEGER.value,integer_value=4))
        req.parameters = [NAV_RCL_ACT, COM_RCL_EXCEPT]
        future = cli_set_params.call_async(req)
        exe.spin_until_future_complete(future)

        # cli_set_params_local_pos = self.create_client(SetParameters, "mavros/local_position/set_parameters")
        # req = SetParameters.Request()
        # LOCAL_TF_SEND = ParameterMsg(name="tf.send",value=ParameterValue(type=Parameter.Type.BOOL.value,bool_value=True))
        # CHILD_FRAME_ID = ParameterMsg(name="tf.child_frame_id",value=ParameterValue(type=Parameter.Type.STRING.value,string_value=self._namespace))
        # req.parameters = [LOCAL_TF_SEND, CHILD_FRAME_ID]
        # future = cli_set_params_local_pos.call_async(req)
        # exe.spin_until_future_complete(future)


    ################
    ## Properties ##
    ################
