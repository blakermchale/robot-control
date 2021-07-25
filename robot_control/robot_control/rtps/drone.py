#!/usr/bin/env python3
# This package
from robot_control.abstract_drone import ADrone
from robot_control.rtps.vehicle import Vehicle
# ROS libraries
import rclpy
from rclpy.executors import MultiThreadedExecutor
# Common packages
from argparse import ArgumentParser


class Drone(ADrone, Vehicle):
    def __init__(self, instance=0):
        super().__init__( instance=instance)
        self.get_logger().debug("Initialized Drone!")

        # uOrb subscribers
        # self._sub_vehicle_land = self.create_subscription(VehicleLandDetected, "RTPS/VehicleLandDetected_PubSubTopic", self._callback_vehicle_land, 1)

    #####################
    ## Control commands
    #####################
    def land(self):
        raise NotImplementedError

    def takeoff(self):
        raise NotImplementedError

    #######################
    ## Checking states
    #######################
    def is_landed(self):
        raise NotImplementedError


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
