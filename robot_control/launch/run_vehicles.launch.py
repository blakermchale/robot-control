#!/usr/bin/env python3
'''
run_vehicles.launch.py

Runs specified number of vehicles in simulation with controller.
'''
from robot_control.launch.structs import ApiType
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

import os
import yaml
import numpy as np

from airsim_utils.generate_settings import get_empty_settings, add_vehicle_settings, write_settings
from airsim_utils.generate_settings import VehicleType as AirSimVehicleType
from robot_control.launch.structs import ScenarioType, SimType

from ros2_utils.launch_manager import LaunchManager

from robot_control.launch.structs import ScenarioType, SimType
from robot_control.launch.run_vehicles import get_team

DEFAULT_SCENARIO = ScenarioType.NONE.name.lower()

def launch_setup(args):
    lm = LaunchManager()
    sim = SimType[args.sim.upper()]
    log_level = args.log_level.upper()
    team_yaml = args.team_yaml
    # NOTE: example team_yaml yaml file in `config/team.yaml`
    if team_yaml != "":
        lm.print_info(f'Using `team_yaml` file "{team_yaml}" to load vehicles.')
    team = get_team(args, args.context)

    # AirSim needs to be started outside since it doesn't follow normal process of (start sim -> spawn vehicle)
    #  instead you must (specify vehicles -> start sim)
    if sim == SimType.AIRSIM:
        ld += generate_settings_from_team(team)

    # Start up environment
    lm.add_include_launch_description("robot_control", "launch/environment.launch.py")

    if sim == SimType.AIRSIM:
        if not os.environ.get("WSL_HOST_IP"):
            raise ValueError("WSL_HOST_IP must be set for AirSim to connect")
        remapped_images = []
        for namespace in team.keys():
            remapped_images += remap_airsim_image(f"/{namespace}/realsense/Scene", f"/{namespace}/realsense/color")
            remapped_images += remap_airsim_image(f"/{namespace}/realsense/DepthPlanar", f"/{namespace}/realsense/aligned_depth_to_color")
        lm.add_action(
            Node(
                package='airsim_ros', executable="airsim_node",
                output='screen',
                arguments=[
                    "--ros-args", "--log-level", f"airsim_ros_wrapper:={log_level}",
                    "-p", f"host_ip:={os.environ['WSL_HOST_IP']}",
                ],
                remappings=remapped_images
            )
        )
            # Node(
            #     package='tf2_ros',
            #     executable='static_transform_publisher',
            #     name='ned_to_enu_pub',
            #     arguments=['0', '0', '0', '1.57', '0', '3.14', 'world_ned', 'world_enu']#, '100']
            # ),
            # Node(
            #     package='airsim_ros_pkgs',
            #     executable='pd_position_controller_simple_node',
            #     name='pid_position_node',
            #     output='screen',
            #     parameters=[{
            #         'update_control_every_n_sec': 0.01,
                    
            #         'kp_x': 0.30,
            #         'kp_y': 0.30,
            #         'kp_z': 0.30,
            #         'kp_yaw': 0.30,
                    
            #         'kd_x': 0.05,
            #         'kd_y': 0.05,
            #         'kd_z': 0.05,
            #         'kd_yaw': 0.05,
                
            #         'reached_thresh_xyz': 0.1,
            #         'reached_yaw_degrees': 5.0
            #     }]
            # )

    # Spawn vehicles
    lm.print_info(f'Spawning vehicles with namespaces "{", ".join(team.keys())}".')
    for spawn_args in team.values():
        spawn_args_str = {k: v if isinstance(v, dict) else str(v) for k, v in spawn_args.items()}
        lm.add_include_launch_description("robot_control", "spawn_vehicle.launch.py", spawn_args_str)
    return lm.describe_sub_entities()


def generate_settings_from_team(team:dict):
    from robot_control.launch.structs import VehicleType
    lm = LaunchManager()
    settings = get_empty_settings()
    for namespace, v in team.items():
        vehicle_type, api = VehicleType[v["vehicle_type"].upper()], ApiType[v["api"].upper()]
        if vehicle_type == VehicleType.DRONE and api == ApiType.MAVROS:
            airsim_type = AirSimVehicleType.PX4MULTIROTOR
        elif vehicle_type == VehicleType.DRONE and api == ApiType.INHERENT:
            airsim_type = AirSimVehicleType.SIMPLEFLIGHT
        else:
            raise NotImplementedError(f"Vehicle type {vehicle_type.name.lower()} and api {api.name.lower()} not supported in AirSim")
        i = v["instance"]
        x, y, z, roll, pitch, yaw = v["x"], v["y"], v["z"], v["roll"], v["pitch"], v["yaw"]
        roll = np.deg2rad(roll)
        pitch = np.deg2rad(pitch)
        yaw = np.deg2rad(yaw)
        if np.isnan(x): x = 0.0
        if np.isnan(y): y = int(i)*2
        if np.isnan(z): z = 0.15
        add_vehicle_settings(settings, namespace, airsim_type, x, y, z, roll, pitch, yaw, hitl=v["hitl"], instance=i)
    write_settings(settings)
    lm.print_info("Wrote settings for AirSim!")
    return lm.describe_sub_entities()


def remap_airsim_image(input_image_ns, output_image_ns):
    return [
        (f"{input_image_ns}", f"{output_image_ns}/image_raw"),
        (f"{input_image_ns}/camera_info", f"{output_image_ns}/camera_info"),
        (f"{input_image_ns}/compressed", f"{output_image_ns}/image_raw/compressed"),
        (f"{input_image_ns}/compressedDepth", f"{output_image_ns}/image_raw/compressedDepth"),
        (f"{input_image_ns}/theora", f"{output_image_ns}/image_raw/theora")
    ]


def generate_launch_description():
    lm = LaunchManager()
    # Launch arguments
    lm.add_arg("team_yaml", "", "Path to yaml file that contains namespaces for a vehicle and their relevant launch args. See examples/ folder.")
    lm.add_arg("scenario", DEFAULT_SCENARIO, "Scenario for robot. Changes types of vehicles, sensors, and environment used.")
    lm.add_arg("nb", "1", "Number of vehicles to spawn.")
    lm.add_arg("namespaces", "[]", "List of namespaces. Will autofill if number spawned is greater than number of items. Ex: ['drone','rover'].")
    lm.add_arg("base_name", "", "Prefix for all vehicles.")
    lm.add_arg("pawn_bp", "", "Pawn blueprint to spawn in AirSim.")
    lm.add_include_launch_args("robot_control", "spawn_vehicle.launch.py", exclude=["namespace", "instance", "log_level", "sim"])
    lm.add_include_launch_args("robot_control", "environment.launch.py")
    # Launch arguments from YAML
    yaml_file = ""
    # NOTE: example ROBOT_CONTROL_CONFIG yaml file in `config/robot_control.yaml`
    if os.environ.get("ROBOT_CONTROL_CONFIG"):
        lm.print_info('Launching using `ROBOT_CONTROL_CONFIG` file instead of passed in arguments.')
        yaml_file = os.environ["ROBOT_CONTROL_CONFIG"]
    lm.add_opaque_function(launch_setup, yaml_file)
    return lm
