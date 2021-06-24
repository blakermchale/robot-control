#!/usr/bin/env python
# This package
from robot_control.abstract_drone import ADrone
from robot_control.airsim.vehicle import Vehicle
from robot_control.airsim.my_multirotor_client import MyMultirotorClient
# ROS libraries
import rclpy
from rclpy.executors import MultiThreadedExecutor
# Common packages
import numpy as np


class Drone(ADrone, Vehicle):
    def __init__(self, log_level="info", instance=0):
        super().__init__(log_level=log_level, instance=instance)
        self._client = MyMultirotorClient(self.namespace)
        print("Drone")

    def update(self):
        self._client.update_state()
        self._position = self._client.get_position()
        self._orientation = self._client.get_orientation()
        super().update()

    def land(self):
        self._client.land()

    def is_landed(self):
        return self._client.is_landed()

    def takeoff(self, alt: float):
        # TODO: switch to using send Z async
        self.set_target(self._position[0], self._position[1], self._position[2] + -1.5)  # it seems like drone actually only goes 1.5 m up instead of 3.0 m
        self._client.takeoff()

    def arm(self):
        self._client.arm()
    
    def disarm(self):
        self._client.disarm()

    def distance_to_target(self):
        return np.linalg.norm(self._position-self._target_position)


def main(args=None):
    rclpy.init(args=args)
    drone = Drone()
    executor = MultiThreadedExecutor()
    rclpy.spin(drone, executor=executor)


if __name__=="__main__":
    main()
