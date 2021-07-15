#!/usr/bin/env python3
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
import os


gazebo_ros = get_package_share_directory("gazebo_ros")

# Arguments with relevant info, type defaults to string
LAUNCH_ARGS = [
    {"name": "gui",             "default": "true",              "description": "Starts gazebo gui"},
    {"name": "server",          "default": "true",              "description": "Starts gazebo server to run simulations in background"},
    {"name": "verbose",         "default": "false",             "description": "Starts gazebo server with verbose outputs"},
    {"name": "world",           "default": "empty.world",       "description": "Gazebo world to load"},
]


def launch_setup(context, *args, **kwargs):
    """Allows declaration of launch arguments within the ROS2 context
    """
    world = LaunchConfiguration("world").perform(context)

    ld = []
    ld.append(
        LogInfo(msg=[
            'Launching ', LaunchConfiguration('world')
        ]),
    )
    # If file is not absolute assume it is a world from PX4
    if not os.path.exists(world):
        # Check for empty variables
        if not os.environ.get("PX4_AUTOPILOT"):
            raise Exception("PX4_AUTOPILOT env variable must be set")
        world = os.path.join(os.environ["PX4_AUTOPILOT"], "Tools", "sitl_gazebo", "worlds", world)
    ld.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [gazebo_ros, os.path.sep, 'launch', os.path.sep, 'gzserver.launch.py']),
            condition=IfCondition(LaunchConfiguration('server')),
            launch_arguments={
                'world': world,
                'verbose': LaunchConfiguration('verbose'),
            }.items(),
        )
    )
    ld.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [gazebo_ros, os.path.sep, 'launch', os.path.sep, 'gzclient.launch.py']),
            condition=IfCondition(LaunchConfiguration('gui'))
        )
    )
    return ld
