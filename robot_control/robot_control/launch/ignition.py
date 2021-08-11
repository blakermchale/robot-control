#!/usr/bin/env python3
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
import os
from robot_control.launch.common import get_local_arguments, combine_names


pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
pkg_robot_ignition = get_package_share_directory('robot_ignition')

# Arguments with relevant info, type defaults to string
LAUNCH_ARGS = [
    {"name": "gui",             "default": "true",              "description": "Starts gazebo gui", "type": "bool"},
    {"name": "server",          "default": "true",              "description": "Starts gazebo server to run simulations in background", "type": "bool"},
    {"name": "verbose",         "default": "false",             "description": "Starts gazebo server with verbose outputs", "type": "bool"},
    {"name": "world",           "default": "empty.sdf",         "description": "Ignition world to load"},
]


def launch_setup(context, *args, **kwargs):
    """Allows declaration of launch arguments within the ROS2 context
    """
    world = LaunchConfiguration("world").perform(context)
    largs = get_local_arguments(LAUNCH_ARGS, context)

    ld = []
    ld.append(
        LogInfo(msg=[
            'Launching ', LaunchConfiguration('world')
        ]),
    )
    # Ignition env variables
    setup()
    # Add args to ign gazebo call
    # world = os.path.join(pkg_robot_ignition, "worlds", world)
    ign_args = f"-r {world}"
    if largs["gui"] and not largs["server"]: ign_args += " -g"
    if largs["server"] and not largs["gui"]: ign_args += " -s"
    if largs["verbose"]: ign_args += " -v"
    # default ignition worlds here https://github.com/ignitionrobotics/ign-gazebo/tree/main/examples/worlds
    ld.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_ign_gazebo, "launch", "ign_gazebo.launch.py")),
            condition=IfCondition(LaunchConfiguration('server')),
            launch_arguments={
                "ign_args": ign_args
            }.items(),
        )
    )
    return ld


def setup():
    if not os.environ.get("PX4_AUTOPILOT"):
        raise Exception("PX4_AUTOPILOT env variable must be set")
    px4_path = os.environ["PX4_AUTOPILOT"]
    px4_ign_path = os.path.join(px4_path, "Tools", "simulation-ignition")
    px4_ign_build_path = os.path.join(px4_path, "build", "px4_sitl_default", "build_ign_gazebo")
    ld_libs = [os.environ.get("LD_LIBRARY_PATH"), px4_ign_build_path]
    os.environ["LD_LIBRARY_PATH"] = combine_names(ld_libs, ":")
    plugins = [os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH"), px4_ign_build_path]
    os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"] = combine_names(plugins, ":")
    resources = [os.environ.get("IGN_GAZEBO_RESOURCE_PATH"), os.path.join(px4_ign_path, "models")]
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = combine_names(resources, ":")
    print(os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"])
