#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Duration
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

from mavros import Client


class MavrosClient(Client):
    def __init__(self):
        super().__init__("ex_mavros", "drone_0/mavros")

        self.current_state = State()
        self.system.subscribe_state(self._cb_state)
        self.pub_setpoint_local = self.setpoint_position.pub_local
        self.cli_arming = self.command.cli_arming
        self.cli_set_mode = self.system.cli_set_mode

    def _cb_state(self, msg: State):
        self.current_state = msg


def main(args=None):
    rclpy.init(args=args)
    ex = MavrosClient()
    executor = MultiThreadedExecutor()
    # Setpoint publishing rate must be faster than 2 Hz
    rate = ex.create_rate(20.0)  # FIXEME: rate locks up ros when sleep is called
    # Wait for connection
    while rclpy.ok() and not ex.current_state.connected:
        rclpy.spin_once(ex)
        ex.get_logger().info("Trying to connect", throttle_duration_sec=2.0)
        # rate.sleep()
    ex.get_logger().info("Connected!")
    pose = PoseStamped()
    pose.pose.position = Point(x=0.0, y=0.0, z=2.0)
    # Send a few setpoints before starting
    i = 100
    while rclpy.ok() and i > 0:
        ex.pub_setpoint_local.publish(pose)
        rclpy.spin_once(ex)
        ex.get_logger().info("Sending initial setpoints", throttle_duration_sec=2.0)
        # rate.sleep()
        i -= 1
    offb_set_mode = SetMode.Request()
    offb_set_mode.custom_mode = "OFFBOARD"  # px4_cmode_map in uas_stringify.cpp
    arm_cmd = CommandBool.Request()
    arm_cmd.value = True
    last_request = ex.get_clock().now()
    while rclpy.ok():
        if ex.current_state.mode != "OFFBOARD" and ex.get_clock().now() - last_request > Duration(seconds=5.0):
            ex.get_logger().info("Sending offboard mode")
            res = ex.cli_set_mode.call(offb_set_mode)
            ex.get_logger().info("sent offboard")
            # if req := ex.cli_set_mode.call(offb_set_mode) and res.mode_sent:
            if res.mode_sent:
                ex.get_logger().info("Offboard enabled")
            last_request = ex.get_clock().now()
        else:
            if not ex.current_state.armed and ex.get_clock().now() - last_request > Duration(seconds=5.0):
                ex.get_logger().info("Sending arm")
                if ex.cli_arming.call(arm_cmd) and arm_cmd.response.success:
                    ex.get_logger().info("Vehicle armed")
                last_request = ex.get_clock().now()
        ex.pub_setpoint_local.publish(pose)
        rclpy.spin_once(ex, executor=executor)
        ex.get_logger().info("Publishing setpoint", throttle_duration_sec=2.0)
        # rate.sleep()
        

if __name__=="__main__":
    main()
