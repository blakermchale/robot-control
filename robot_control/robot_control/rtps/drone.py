#!/usr/bin/env python
# This package
from robot_control.abstract_drone import ADrone
from robot_control.rtps.vehicle import Vehicle
# ROS libraries
import rclpy
from rclpy.executors import MultiThreadedExecutor


class Drone(ADrone, Vehicle):
    def __init__(self, log_level="info", instance=0):
        super().__init__(log_level=log_level, instance=instance)
        print("Drone")


def main(args=None):
    rclpy.init(args=args)
    drone = Drone()
    executor = MultiThreadedExecutor()
    rclpy.spin(drone, executor=executor)


if __name__=="__main__":
    main()
