#!/usr/bin/env python

from robot_control.abstract_vehicle import AVehicle


class Vehicle(AVehicle):
    def __init__(self, log_level="info", instance=0):
        super().__init__(log_level=log_level, instance=instance)
        print("Vehicle")
