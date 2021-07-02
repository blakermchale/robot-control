#!/usr/bin/env python
# This package
from robot_control.abstract_drone import ADrone
from robot_control.mavros.vehicle import Vehicle
# ROS libraries
import rclpy
from rclpy.executors import MultiThreadedExecutor
# Common packages
from argparse import ArgumentParser


class Drone(ADrone, Vehicle):
    def __init__(self, log_level="info", instance=0):
        super().__init__(log_level=log_level, instance=instance)
        self.get_logger().debug("Initialized Drone!")


def main(args=None):
    rclpy.init(args=args)
    # Setup argument parsing
    parser = ArgumentParser()
    parser.add_argument("--log-level", default='info', choices=["info", "debug", "warn", "error", "fatal"], help='ros log level')
    parser.add_argument("-i", "--instance", default=0, type=int, help="Instance of vehicle.")
    args, _ = parser.parse_known_args()
    # Spin drone
    drone = Drone(instance=args.instance)
    executor = MultiThreadedExecutor()
    rclpy.spin(drone, executor=executor)


if __name__=="__main__":
    main()
