#!/usr/bin/env python
# This package
from robot_control import Frame
from robot_control.ignition.vehicle import Vehicle
# ROS libraries
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
# ROS interfaces
from geometry_msgs.msg import Twist
from ros2_utils import convert_axes_from_msg, AxesFrame
from ros2_utils.math import angular_dist, wrap_to_pi
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
# Our libraries
from ..utils.pid_position_controller import PIDPositionController, XYZYaw
# Common packages
import numpy as np
from argparse import ArgumentParser


class Rover(Vehicle):
    def __init__(self, instance=0):
        super().__init__(instance=instance)
        # Override base variables
        self._wait_moved = Duration(seconds=10)
        # Internal vars
        self.target_valid = False  # Used for computing move to position
        # TODO: create parameter bridges for sensor msgs like GPS
        # Subscribers
        self._sub_ign_odom = self.create_subscription(Odometry, "_ign/odom", self._callback_ign_odom, 10)
        # Publishers
        self._pub_ign_twist = self.create_publisher(Twist, "_ign/gazebo/command/twist", 1)

        # ROS parameters
        self.declare_parameter("posctl.x.p", 0.8)
        self.declare_parameter("posctl.x.d", 0.1)
        self.declare_parameter("posctl.y.p", 0.8)
        self.declare_parameter("posctl.y.d", 0.1)
        self.declare_parameter("posctl.z.p", 1.0)
        self.declare_parameter("posctl.z.d", 0.1)
        self.declare_parameter("posctl.yaw.p", 1.0)
        self.declare_parameter("posctl.yaw.d", 0.1)
        self.declare_parameter("posctl.max_vel_horz_abs", 5.0)
        self.declare_parameter("posctl.max_vel_vert_abs", 3.0)
        self.declare_parameter("posctl.max_yaw_rate", 10.0)
        posctl = self.get_parameters_by_prefix("posctl")
        posctl = {k: v.value for k, v in posctl.items()}
        kp = np.asfarray([posctl["x.p"], posctl["y.p"], posctl["z.p"], posctl["yaw.p"]])
        kd = np.asfarray([posctl["x.d"], posctl["y.d"], posctl["z.d"], posctl["yaw.d"]])
        self.pid_posctl = PIDPositionController(kp, kd, posctl["max_vel_horz_abs"], posctl["max_vel_vert_abs"], posctl["max_yaw_rate"])

        self.get_logger().info("Ignition rover initialized")

    def update(self):
        if self.target_valid and not self.reached_target(0.3):
            self.pid_posctl.set_current(self.position.x, self.position.y, self.position.z, self.euler.z)
            # self.pid_posctl.set_target(self.target.position.x, self.target.position.y, self.target.position.z, self.target.euler.z)
            state = "none"
            if self.reached_target(tolerance=0.5, check_yaw=False):
                self.pid_posctl.set_target(self.target.position.x, self.target.position.y, self.target.position.z, self.target.euler.z)
                state = "reached"
            elif not self.aligned_target():
                forward_ang = self.angle_to_target()
                backward_ang = forward_ang + np.pi
                min_idx = np.argmin(np.abs([angular_dist(self.euler.z, forward_ang), angular_dist(self.euler.z, backward_ang)]))
                ang = forward_ang if min_idx == 0 else backward_ang
                self.pid_posctl.set_target(self.position.x, self.position.y, self.position.z, ang)
                state = "not aligned"
            elif self.aligned_target():
                forward_ang = self.angle_to_target()
                backward_ang = angular_dist(self.euler.z, forward_ang + np.pi)
                min_idx = np.argmin(np.abs([angular_dist(self.euler.z, forward_ang), angular_dist(self.euler.z, backward_ang)]))
                ang = forward_ang if min_idx == 0 else backward_ang
                self.pid_posctl.set_target(self.target.position.x, self.target.position.y, self.target.position.z, ang)
                state = "aligned"
            else:
                self.get_logger().error("Invalid update state for target")
            # self.get_logger().info(f"{state}, {self.pid_posctl.curr_error}")
            # self.get_logger().error(f"------------CURRENT-----------\n{self.pose}", throttle_duration_sec=0.5)
            # self.get_logger().error(f"------------TARGET-----------\n{self.target}", throttle_duration_sec=0.5)
            vel_cmd = self.pid_posctl.update()
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
    def halt(self):
        self._reset_pidctl()
        return True

    def send_waypoint(self, x: float, y: float, z: float, heading: float, frame: Frame = Frame.LOCAL_NED):
        try:
            x, y, z, roll, pitch, heading = self.convert_position_frame(x, y, z, 0, 0, heading, frame, Frame.LOCAL_NED)
        except Exception as e:
            self.get_logger().error(str(e))
            return False
        self.target_valid = True
        self.set_target(x, y, self.position.z, heading)
        return True

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float, frame: Frame = Frame.FRD):
        vx, vy, vz, roll_rate, pitch_rate, yaw_rate = self.convert_velocity_frame(vx, vy, vz, 0, 0, yaw_rate, frame, Frame.FRD)
        vy = 0.0  # rover can't use y or z velocities in FRD
        vz = 0.0
        if np.isnan([vx, vy, vz, roll_rate, pitch_rate, yaw_rate]).any():
            self.get_logger().warn(f"NaN velocity attempted, not sending - x: {vx}, y: {vy}, z: {vz}, yaw: {yaw_rate}", throttle_duration_sec=0.1)
            return False
        self.target_odom.lin_vel = [vx, vy, vz]
        self.target_odom.ang_vel = [roll_rate, pitch_rate, yaw_rate]
        # self.get_logger().info(f"x: {vx}, y: {vy}, z: {vz}, yaw: {yaw_rate}", throttle_duration_sec=0.1)
        return True

    #######################
    ## Checking states
    #######################
    def is_armed(self):
        return True

    ######################
    ## Subscribers
    ######################
    def _callback_ign_odom(self, msg: Odometry):
        self.odom.set_msg(convert_axes_from_msg(msg, AxesFrame.RHAND, AxesFrame.URHAND))

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
    rover = Rover(instance=args.instance)
    executor = MultiThreadedExecutor()
    rclpy.spin(rover, executor=executor)


if __name__=="__main__":
    main()
