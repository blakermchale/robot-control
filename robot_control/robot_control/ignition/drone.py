#!/usr/bin/env python
# This package
from robot_control import Frame
from robot_control.abstract_drone import ADrone
from robot_control.ignition.vehicle import Vehicle
# ROS libraries
import rclpy
from rclpy.executors import MultiThreadedExecutor
# ROS interfaces
from geometry_msgs.msg import Twist
from ros2_utils import convert_axes_from_msg, AxesFrame
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
# Our libraries
from ..utils.pid_position_controller import PIDPositionController, XYZYaw
# Common packages
import numpy as np
from argparse import ArgumentParser


class Drone(ADrone, Vehicle):
    def __init__(self, instance=0):
        super().__init__(instance=instance)
        # Internal vars
        self._rotor_speeds = np.asfarray([np.nan, np.nan, np.nan, np.nan])
        self.target_valid = False  # Used for computing move to position
        # TODO: create parameter bridges for sensor msgs like GPS
        # Subscribers
        self._sub_ign_joint_state = self.create_subscription(JointState, "_sim/joint_state", self._callback_ign_joint_state, 10)
        self._sub_ign_odom = self.create_subscription(Odometry, "_sim/odom", self._callback_ign_odom, 10)
        # Publishers
        self._pub_ign_twist = self.create_publisher(Twist, "_sim/command/twist", 1)
        self._pub_ign_enable = self.create_publisher(Bool, "_sim/enable", 1)

        # ROS parameters
        self.declare_parameter("posctl.x.p", 0.7)
        self.declare_parameter("posctl.x.i", 0.0)
        self.declare_parameter("posctl.x.d", 0.1)
        self.declare_parameter("posctl.y.p", 0.7)
        self.declare_parameter("posctl.y.i", 0.0)
        self.declare_parameter("posctl.y.d", 0.1)
        self.declare_parameter("posctl.z.p", 0.7)
        self.declare_parameter("posctl.z.i", 0.005)
        self.declare_parameter("posctl.z.d", 0.1)
        self.declare_parameter("posctl.yaw.p", 1.0)
        self.declare_parameter("posctl.yaw.i", 0.0)
        self.declare_parameter("posctl.yaw.d", 0.1)
        self.declare_parameter("posctl.max_vel_horz_abs", 5.0)
        self.declare_parameter("posctl.max_vel_vert_abs", 3.0)
        self.declare_parameter("posctl.max_yaw_rate", 1.0)
        posctl = self.get_parameters_by_prefix("posctl")
        posctl = {k: v.value for k, v in posctl.items()}
        kp = np.asfarray([posctl["x.p"], posctl["y.p"], posctl["z.p"], posctl["yaw.p"]])
        ki = np.asfarray([posctl["x.i"], posctl["y.i"], posctl["z.i"], posctl["yaw.i"]])
        kd = np.asfarray([posctl["x.d"], posctl["y.d"], posctl["z.d"], posctl["yaw.d"]])
        self.pid_posctl = PIDPositionController(kp, ki, kd, posctl["max_vel_horz_abs"], posctl["max_vel_vert_abs"], posctl["max_yaw_rate"])

        self.get_logger().info("Ignition drone initialized")

    def update(self):
        if self.target_valid and not self.reached_target(0.1):
            self.pid_posctl.set_current(self.position.x, self.position.y, self.position.z, self.euler.z)
            self.pid_posctl.set_target(self.target.position.x, self.target.position.y, self.target.position.z, self.target.euler.z)
            # self.get_logger().error(f"------------CURRENT-----------\n{self.pose}", throttle_duration_sec=0.5)
            # self.get_logger().error(f"------------TARGET-----------\n{self.target}", throttle_duration_sec=0.5)
            vel_cmd = self.pid_posctl.update()
            # self.get_logger().error(f"{self.pid_posctl.curr_error}")
            # self.get_logger().info(f"x: {vel_cmd.x}, y: {vel_cmd.y}, z: {vel_cmd.z}, yaw: {vel_cmd.yaw}", throttle_duration_sec=1.0)
            self.send_velocity(vel_cmd.x, vel_cmd.y, vel_cmd.z, vel_cmd.yaw, Frame.LOCAL_NED)
        elif self.reached_target(0.001) and self.target_valid:
            self._reset_pidctl()
        # Publish velocity constantly
        msg = Twist()
        # Flip from NED or FRD to ignition frame
        msg = self.target_odom.twist.get_msg()
        msg = convert_axes_from_msg(msg, AxesFrame.URHAND, AxesFrame.RHAND)
        self._pub_ign_twist.publish(msg)
        super().update()

    #####################
    ## Control commands
    #####################
    # TODO: how should return flag be set for arm/disarm?
    def arm(self):
        self.enable_control(True)
        self._reset_pidctl()
        return True
    
    def disarm(self):
        self._reset_pidctl()
        self.enable_control(False)
        return True

    def kill(self):
        self._reset_pidctl()
        self.enable_control(False)
        return True

    def halt(self):
        self._reset_pidctl()
        return True

    def land(self):
        self._reset_pidctl()
        self.send_velocity(0.0, 0.0, 1.0, 0.0)
        return True

    def takeoff(self, alt: float):
        self.target_valid = True
        self.send_waypoint(self.position.x, self.position.y, self.position.z + -alt, self.euler.z, Frame.LOCAL_NED)
        return True

    def send_waypoint(self, x: float, y: float, z: float, heading: float, frame: Frame = Frame.LOCAL_NED):
        try:
            x, y, z, roll, pitch, heading = self.convert_position_frame(x, y, z, 0, 0, heading, frame, Frame.LOCAL_NED)
        except Exception as e:
            self.get_logger().error(str(e))
            return False
        self.target_valid = True
        self.set_target(x, y, z, heading)
        return True

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float, frame: Frame = Frame.FRD):
        vx, vy, vz, roll_rate, pitch_rate, yaw_rate = self.convert_velocity_frame(vx, vy, vz, 0, 0, yaw_rate, frame, Frame.FRD)
        self.target_odom.lin_vel = [vx, vy, vz]
        self.target_odom.ang_vel = [roll_rate, pitch_rate, yaw_rate]
        # self.get_logger().info(f"x: {vx}, y: {vy}, z: {vz}, yaw: {yaw_rate}", throttle_duration_sec=0.1)
        return True

    def enable_control(self, enable: bool):
        """Enables control of multicopter through system plugin.

        Args:
            enable (bool): Flag that enables control.
        """
        msg = Bool()
        msg.data = enable
        self._pub_ign_enable.publish(msg)

    #######################
    ## Checking states
    #######################
    def is_landed(self):
        lnd_max_horizontal = 0.0
        lnd_max_vertical = 0.0
        # Check that no vertical or horizontal velocity is happening
        horz_vel = np.linalg.norm(self.lin_vel[:2])
        vert_vel = np.abs(self.lin_vel.z)
        landed = horz_vel <= lnd_max_horizontal and vert_vel <= lnd_max_vertical
        # landed = landed and self._last_vel_cmd.vz > 0  # Command doesn't match found velocity
        landed = landed and self.position.z > -0.06  # TODO: don't make assumption that floor is 0, use collision sensor
        # self.get_logger().debug(f"Horz: {horz_vel}, Vert: {vert_vel}, landed: {landed}", throttle_duration_sec=1.0)
        # TODO: check rotor speeds and assume landed if off and orientation is not upside down
        # TODO: Landed should check for more than velocity since ignition odom gives back 0 when flying
        return landed or not self.is_armed()

    def is_armed(self):
        armed = not np.isclose(self._rotor_speeds, 0).all() or not np.isnan(self._rotor_speeds).all()
        # self.get_logger().debug(f"Rotor speeds: {self._rotor_speeds}, Armed: {armed}", throttle_duration_sec=2.0)
        return armed

    ######################
    ## Subscribers
    ######################
    def _callback_ign_joint_state(self, msg: JointState):
        # TODO: don't make assumption that all joints are rotors
        self._rotor_speeds = np.asfarray(msg.velocity)
    
    def _callback_ign_odom(self, msg: Odometry):
        self.odom.set_msg(convert_axes_from_msg(msg, AxesFrame.RHAND, AxesFrame.URHAND))

    def _abort_arm_takeoff(self):
        self._reset_pidctl()

    def _abort_go_waypoint(self):
        self._reset_pidctl()

    def _reset_pidctl(self):
        self.target_valid = False
        self.send_velocity(0., 0., 0., 0., Frame.FRD)

    def _pre_callback_velocity(self):
        self.target_valid = False


def main(args=None):
    rclpy.init(args=args)
    # Setup argument parsing
    parser = ArgumentParser()
    parser.add_argument("-i", "--instance", default=0, type=int, help="Instance of vehicle.")
    args, _ = parser.parse_known_args()
    # Spin drone
    drone = Drone(instance=args.instance)
    executor = MultiThreadedExecutor()
    rclpy.spin(drone, executor=executor)


if __name__=="__main__":
    main()
