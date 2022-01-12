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
]


def launch_setup(context, *args, **kwargs):
    """Allows declaration of launch arguments within the ROS2 context
    """
    args = get_local_arguments(LAUNCH_ARGS, context)

    # Check for empty variables
    if not os.environ.get("PX4_AUTOPILOT"):
        raise Exception("PX4_AUTOPILOT env variable must be set")

    ld = []

    # Choose base name according to vehicle type
    sim = SimType[args['sim'].upper()]

    # Establish log level
    log_level = args["log_level"].upper()

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
                    'verbose':args['verbose'],
                    'gui': args['gui'],
                    'world': args['gz_world']
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
                    'verbose':args['verbose'],
                    'gui': args['gui'],
                    'world': args['ign_world']
                }.items(),
            )
        )
    elif sim == SimType.AIRSIM:
        # NOTE: this needs to be called after vehicles have been added to the config
        raise NotImplementedError("AirSim environment should be started from outside")
        ld += generate_airsim(args["hitl"], args["nb"], args["pawn_bp"], namespaces, args["environment"], log_level)
        ld.append(
            Node(
                package='airsim_ros', executable="airsim_node",
                output='screen',
                arguments=[
                    "--ros-args", "--log-level", f"airsim_ros_wrapper:={log_level}"
                ],
            ),
        )

    return ld
