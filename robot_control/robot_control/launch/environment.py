#!/usr/bin/env python3
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os
from glob import glob

from airsim_utils.generate_settings import create_settings, DEFAULT_PAWN_BP
from airsim_utils.generate_settings import VehicleType as AirSimVehicleType
from airsim_utils.run_environment import run_environment
from ros2_utils.launch import get_local_arguments
from robot_control.launch.ignition import setup as setup_ignition
from robot_control.launch.gazebo import setup as setup_gazebo

from robot_control.launch.structs import SimType, ROBOT_CONTROL_PKG


# Arguments with relevant info, type defaults to string
LAUNCH_ARGS = [
    {"name": "sim",             "default": "gazebo",            "description": "Simulation to use.",
        "choices": [e.name.lower() for e in SimType]},
    {"name": "gui",             "default": "true",              "description": "Starts gazebo gui"},
    {"name": "verbose",         "default": "false",             "description": "Starts gazebo server with verbose outputs"},
    {"name": "gz_world",        "default": "empty.world",       "description": "Gazebo world to load"},
    {"name": "ign_world",       "default": "empty.sdf",         "description": "Ignition world to load"},
    {"name": "log_level",       "default": "debug",             "description": "Sets log level of ros nodes."},
    {"name": "environment",     "default": "",                  "description": "Path to executable for running AirSim environment."},
]


def launch_setup(context, *args, **kwargs):
    """Allows declaration of launch arguments within the ROS2 context
    """
    largs = get_local_arguments(LAUNCH_ARGS, context)

    # Check for empty variables
    if not os.environ.get("PX4_AUTOPILOT"):
        raise Exception("PX4_AUTOPILOT env variable must be set")

    ld = []

    # Choose base name according to vehicle type
    sim = SimType[largs['sim'].upper()]

    # Establish log level
    log_level = largs["log_level"].upper()

    # Start simulator
    if sim == SimType.GAZEBO:
        setup_gazebo()
        ld.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [ROBOT_CONTROL_PKG, os.path.sep, 'launch',
                        os.path.sep, 'gazebo.launch.py']
                ),
                launch_arguments={
                    'verbose':largs['verbose'],
                    'gui': largs['gui'],
                    'world': largs['gz_world']
                }.items(),
            )
        )
    elif sim == SimType.IGNITION:
        setup_ignition()
        ld.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [ROBOT_CONTROL_PKG, os.path.sep, 'launch',
                        os.path.sep, 'ignition.launch.py']
                ),
                launch_arguments={
                    'verbose':largs['verbose'],
                    'gui': largs['gui'],
                    'world': largs['ign_world']
                }.items(),
            )
        )
    elif sim == SimType.AIRSIM:
        env = largs["environment"]
        if not env:
            raise Exception(f"AirSim environment must be valid not '{env}'")
        run_env = run_environment(env=env)
        # FIXME: resolve run_env flag always being false
        # if not run_env:
        #     ld.append(
        #         LogInfo(msg=[
        #             'Exiting early because run environment failed.'
        #         ])
        #     )
        #     return ld

    return ld
