#!/usr/bin/env python3
'''
run_vehicles.launch.py

Runs specified number of vehicles in simulation with controller.
'''
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from robot_control.launch.run_vehicles import launch_setup, LAUNCH_ARGS
from robot_control.launch.common import get_launch_arguments


# https://github.com/colcon/colcon-core/issues/169
def generate_launch_description():   
    launch_description = []
    launch_description += get_launch_arguments(LAUNCH_ARGS)
    launch_description += [OpaqueFunction(function = launch_setup)] 
    return LaunchDescription(launch_description)
