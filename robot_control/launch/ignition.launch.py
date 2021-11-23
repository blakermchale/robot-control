#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from robot_control.launch.ignition import launch_setup, LAUNCH_ARGS
from ros2_utils.launch import get_launch_arguments


def generate_launch_description():
    """Launch this launch file."""
    launch_description = []
    launch_description += get_launch_arguments(LAUNCH_ARGS)
    launch_description += [OpaqueFunction(function=launch_setup)]
    return LaunchDescription(launch_description)
