#!/usr/bin/env python3

from robot_control.abstract_vehicle import AVehicle


class Vehicle(AVehicle):
    def __init__(self, instance=0):
        super().__init__(instance=instance)
        self.get_logger().debug("Vehicle initialized")
