#!/usr/bin/env python
# This package
from robot_control import Frame, Axis
from robot_control.abstract_drone import ADrone
from robot_control.ignition.vehicle import Vehicle
# ROS libraries
import rclpy
from rclpy.executors import MultiThreadedExecutor
# ROS interfaces
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
# Our libraries
from robot_control.utils.pid_position_controller import PIDPositionController
# Common packages
import numpy as np
from argparse import ArgumentParser


class Drone(ADrone, Vehicle):
    def __init__(self, instance=0):
        super().__init__(instance=instance)
        # Internal vars
        self._rotor_speeds = np.asfarray([np.nan, np.nan, np.nan, np.nan])
        # TODO: create parameter bridges for sensor msgs like GPS
        # Subscribers
        self._sub_ign_joint_state = self.create_subscription(JointState, "_ign/joint_state", self._callback_ign_joint_state, 10)
        self._sub_ign_odom = self.create_subscription(Odometry, "_ign/odom", self._callback_ign_odom, 10)
        # Publishers
        self._pub_ign_twist = self.create_publisher(Twist, "_ign/gazebo/command/twist", 1)
        self._pub_ign_enable = self.create_publisher(Bool, "_ign/enable", 1)

        # ROS parameters
        self.declare_parameter("posctl.x.p", 1.0)
        self.declare_parameter("posctl.x.d", 0.0)
        self.declare_parameter("posctl.y.p", 1.0)
        self.declare_parameter("posctl.y.d", 0.0)
        self.declare_parameter("posctl.z.p", 1.0)
        self.declare_parameter("posctl.z.d", 0.0)
        self.declare_parameter("posctl.yaw.p", 1.0)
        self.declare_parameter("posctl.yaw.d", 0.0)
        self.declare_parameter("posctl.max_vel_horz_abs", 5.0)
        self.declare_parameter("posctl.max_vel_vert_abs", 3.0)
        self.declare_parameter("posctl.max_yaw_rate", 3.0)
        posctl = self.get_parameters_by_prefix("posctl")
        kp = np.asfarray([posctl["x"]["p"], posctl["y"]["p"], posctl["z"]["p"], posctl["yaw"]["p"]])
        kd = np.asfarray([posctl["x"]["d"], posctl["y"]["d"], posctl["z"]["d"], posctl["yaw"]["d"]])
        self.pid_posctl = PIDPositionController(kp, kd, posctl["max_vel_horz_abs"], posctl["max_vel_vert_abs"], posctl["max_yaw_rate"])

        self.get_logger().info("Ignition drone initialized")

    def update(self):
        self.pid_posctl.set_current(self.position.x, self.position.y, self.position.z, self.euler.z)
        self.pid_posctl.set_target(self.target_position.x, self.target_position.y, self.target_position.z, self.euler.z)
        super().update()

    #####################
    ## Control commands
    #####################
    # TODO: how should return flag be set for arm/disarm?
    def arm(self):
        # self.enable_control(True)
        self.send_velocity(0.0, 0.0, 0.0, 0.0)
        return True
    
    def disarm(self):
        self.send_velocity(0.0, 0.0, 0.0, 0.0)
        self.enable_control(False)
        return True

    def halt(self):
        raise NotImplementedError

    def land(self):
        raise NotImplementedError

    def takeoff(self, alt: float):
        raise NotImplementedError

    def send_waypoint(self, x: float, y: float, z: float, heading: float, frame: int):
        raise NotImplementedError

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float, frame: int = Frame.BODY_NED):
        msg = Twist()
        if frame == Frame.LOCAL_NED:
            raise NotImplementedError
        elif frame == Frame.BODY_NED:
            msg.linear.x = vx
            msg.linear.y = vy
            msg.linear.z = vz
            msg.angular.z = yaw_rate
            self._pub_ign_twist.publish(msg)
        else:
            self.get_logger().error(f"Frame {frame.name} is not supported")

    def enable_control(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self._pub_ign_enable.publish(msg)

    #######################
    ## Checking states
    #######################
    def is_landed(self):
        # TODO: check rotor speeds and assume landed if off and orientation is not upside down
        # return landed
        return not self.is_armed()

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
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        lin_vel = msg.twist.twist.linear
        ang_vel = msg.twist.twist.angular
        self.position.v3 = [position.x, -position.y, -position.z]
        self.orientation.v4 = orientation
        self.lin_vel.v3 = lin_vel
        self.ang_vel.v3 = ang_vel


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
