#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory

import os
from ros2_utils.launch import combine_names


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
