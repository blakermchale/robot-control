#!/usr/bin/env python3
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
import os
from ros2_utils.launch import combine_names


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


def setup():
    if not os.environ.get("PX4_AUTOPILOT"):
        raise Exception("PX4_AUTOPILOT env variable must be set")
    px4_path = os.environ["PX4_AUTOPILOT"]
    px4_gazebo_path = os.path.join(px4_path, "Tools", "sitl_gazebo")

    robot_gazebo_path = get_package_share_directory("robot_gazebo")
    nuav_gazebo_path = get_package_share_directory("nuav_gazebo")

    px4_gazebo_build_path = os.path.join(px4_path, "build", "px4_sitl_default", "build_gazebo")
    ld_libs = [
        os.environ.get("LD_LIBRARY_PATH"),
        px4_gazebo_build_path
    ]
    os.environ["LD_LIBRARY_PATH"] = combine_names(ld_libs, ":")
    plugins = [
        os.environ.get("GAZEBO_PLUGIN_PATH"),
        px4_gazebo_build_path
    ]
    os.environ["GAZEBO_PLUGIN_PATH"] = combine_names(plugins, ":")
    models = [
        os.environ.get("GAZEBO_MODEL_PATH"),
        os.path.join(px4_gazebo_path, "models"),
        os.path.join(robot_gazebo_path, "models"),
        os.path.join(nuav_gazebo_path, "models")
    ]
    os.environ["GAZEBO_MODEL_PATH"] = combine_names(models, ":")
    resources = [
        os.environ.get("GAZEBO_RESOURCE_PATH"),
        os.path.join(px4_gazebo_path, "worlds"),
        os.path.join(robot_gazebo_path, "worlds"),
        os.path.join(nuav_gazebo_path, "worlds")
    ]
    os.environ["GAZEBO_RESOURCE_PATH"] = combine_names(resources, ":")
