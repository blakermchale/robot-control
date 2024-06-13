#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory

import os
from ros2_utils.launch_manager import LaunchManager
from ros2_utils.launch import combine_names

# TODO: need to change this to support gz sim instead of ign
# https://docs.px4.io/main/en/sim_gazebo_gz/


def launch_setup(args):
    world = args.world
    server = args.server
    verbose = args.verbose

    lm = LaunchManager()

    lm.print_info(f"Launching {world}")

    # Ignition env variables
    setup()

    # Add args to ign gazebo call
    gz_args = f"-r {world}"
    if server: gz_args += " -s"
    if verbose: gz_args += " -v"
    # default ignition worlds here https://github.com/ignitionrobotics/ign-gazebo/tree/main/examples/worlds
    launch_args = {
        "gz_args": gz_args
    }
    # lm.add_arg("hi", "h")
    lm.add_include_launch_description("ros_gz_sim", "launch/gz_sim.launch.py", launch_args)
    return lm.describe_sub_entities()


def setup():
    if not os.environ.get("PX4_AUTOPILOT"):
        raise Exception("PX4_AUTOPILOT env variable must be set")
    px4_path = os.environ["PX4_AUTOPILOT"]
    px4_gz_path = os.path.join(px4_path, "Tools", "simulation", "gz")
    # px4_gz_build_path = os.path.join(px4_path, "build", "px4_sitl_default", "build_ign_gazebo")

    robot_ignition_path = get_package_share_directory("robot_ignition")

    # ld_libs = [
    #     os.environ.get("LD_LIBRARY_PATH"),
    #     # px4_gz_build_path,
    # ]
    # os.environ["LD_LIBRARY_PATH"] = combine_names(ld_libs, ":")

    # plugins = [
    #     os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH"),
    #     # px4_ign_build_path,
    # ]
    # os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"] = combine_names(plugins, ":")

    resources = [
        os.environ.get("GZ_SIM_RESOURCE_PATH"),
        os.path.join(px4_gz_path, "models"),
        os.path.join(robot_ignition_path, "models"),
        os.path.join(px4_gz_path, "worlds"),
        os.path.join(robot_ignition_path, "worlds"),
    ]
    os.environ["GZ_SIM_RESOURCE_PATH"] = combine_names(resources, ":")


def generate_launch_description():
    """Launch this launch file."""
    lm = LaunchManager()
    lm.add_arg("world", "empty.sdf", "Ignition world to load")
    lm.add_arg("server", False, "Starts gazebo server to run simulations in background")
    lm.add_arg("verbose", True, "Starts gazebo server with verbose outputs")
    lm.add_opaque_function(launch_setup)
    return lm
