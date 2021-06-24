#!/usr/bin/env python3

from launch import LaunchDescription, launch_description
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir

from ament_index_python.packages import get_package_share_directory
import os


gazebo_ros = get_package_share_directory("gazebo_ros")


def generate_launch_description():
    """Launch the example.launch.py launch file."""
    launch_description = []
    launch_description += get_launch_arguments()
    launch_description += [
        LogInfo(msg=[
            'Launching ', LaunchConfiguration('world')
        ]),
    ]
    launch_description += [OpaqueFunction(function=launch_setup)]
    return LaunchDescription(launch_description)


def get_launch_arguments():
    return [
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Starts gazebo gui',
        ),
        DeclareLaunchArgument(
            'server',
            default_value='true',
            description='Starts gazebo server to run simulations in background',
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Starts gazebo server with verbose outputs',
        ),
        DeclareLaunchArgument(
            'world',
            default_value="base.world",
            description='Starts gazebo server with world file.',
        ),
    ]


def launch_setup(context, *args, **kwargs):
    """Allows declaration of launch arguments within the ROS2 context
    """
    
    ld = []
    ld.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [gazebo_ros, os.path.sep, 'launch', os.path.sep, 'gzserver.launch.py']),
            condition=IfCondition(LaunchConfiguration('server')),
            launch_arguments={
                # 'world': [ardupilot_gazebo, os.path.sep, "worlds", os.path.sep, LaunchConfiguration('world')],
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
