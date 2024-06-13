#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from ros2_utils.launch import get_launch_arguments
from ros2_utils.launch_manager import LaunchManager
from robot_control.launch.structs import SimType, ROBOT_CONTROL_PKG
from airsim_utils.run_environment import run_environment
import os
from robot_control.launch.ignition import setup as setup_ignition


def launch_setup(args):
    # Check for empty variables
    if not os.environ.get("PX4_AUTOPILOT"):
        raise Exception("PX4_AUTOPILOT env variable must be set")

    lm = LaunchManager()
    # Choose base name according to vehicle type
    sim = SimType[args.sim.upper()]

    # Establish log level
    log_level = args.log_level.upper()

    # Start simulator
    if sim == SimType.GAZEBO:
        launch_arguments = {
                    'verbose':args.verbose,
                    'gui': args.gui,
                    'world': args.gz_world
                }
        # setup_gazebo()
        lm.add_include_launch_description("robot_control", "gazebo.launch.py", launch_arguments)
    elif sim == SimType.IGNITION:
        setup_ignition()
        launch_arguments={
            'verbose':args.verbose,
            'gui': args.gui,
            'world': args.ign_world,
        }
        lm.add_include_launch_description("robot_control", "ignition.launch.py", launch_arguments)
    elif sim == SimType.AIRSIM:
        env = args.environment
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
    return lm.describe_sub_entities()


def generate_launch_description():
    """Launch this launch file."""
    lm = LaunchManager()
    lm.add_arg("sim", "ignition", "Simulation to use.", [e.name.lower() for e in SimType])
    lm.add_arg("gui", "True", "Starts Gazebo GUI")
    lm.add_arg("verbose", "False", "Starts Gazebo server")
    lm.add_arg("gz_world", "empty.world", "Gazebo world to load")
    lm.add_arg("ign_world", "empty.sdf", "Ignition world to laod")
    lm.add_arg("log_level", "debug", "Sets log level of ROS nodes.")
    lm.add_arg("environment", "", "Path to executable for running AirSim environment")
    lm.add_opaque_function(launch_setup)
    return lm
