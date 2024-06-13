#!/usr/bin/env python3
from robot_control.launch.structs import ApiType
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import LogInfo

import os
import yaml
import numpy as np

from airsim_utils.generate_settings import get_empty_settings, add_vehicle_settings, write_settings
from airsim_utils.generate_settings import VehicleType as AirSimVehicleType
from ros2_utils.launch_manager import get_launch_description


def get_team(args, context):
    """Gets a dictionary of all vehicles being spawned and their fields."""
    default_spawn_args = {}
    spawn_launch = get_launch_description("robot_control", "spawn_vehicle.launch.py")
    for a in spawn_launch.get_launch_arguments():
        value = getattr(args, a.name, None)
        if value:
            default_spawn_args[a.name] = value
    team_yaml = args.team_yaml
    team = {}
    if team_yaml != "":
        param_file = ParameterFile(
            param_file=team_yaml,
            allow_substs=True)
        param_file_path = param_file.evaluate(context)
        with open(param_file_path, 'r') as f:
            config_vals = yaml.load(f, Loader=yaml.FullLoader)
        # Overrides passed launch args with config files
        for i, (namespace, v) in enumerate(config_vals.items()):
            spawn_args = default_spawn_args.copy()
            spawn_args["namespace"] = namespace
            spawn_args["instance"] = str(i)
            if not isinstance(v, dict):
                raise ValueError(f"'{namespace}' invalid. Team yaml must contain a nested dictionary associated with each namespace.")
            for name, value in v.items():
                # Forces it to be a spawn launch arg
                # if name in spawn_args:
                spawn_args[name] = value
            team[namespace] = spawn_args
    else:
        # Pre-process namespaces
        namespaces = extract_namespaces(args)
        for i, namespace in enumerate(namespaces):
            spawn_args = default_spawn_args.copy()
            spawn_args["namespace"] = namespace
            spawn_args["instance"] = str(i)
            team[namespace] = spawn_args
    return team


def generate_settings_from_team(team:dict):
    from robot_control.launch.spawn_vehicle import VehicleType
    ld = []
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
    ld += [
        LogInfo(msg=["Wrote settings for AirSim!"])
    ]
    return ld


def extract_namespaces(args):
    if args.base_name == "":
        args.base_name = args.vehicle_type
    namespaces = args.namespaces
    len_ns = len(namespaces)
    if args.nb > len_ns:
        gen_num = args.nb - len_ns  # amount to generate
        for i in range(gen_num):
            namespaces.append(f"{args.base_name}_{len_ns+i}")
    if len(namespaces) != len(set(namespaces)):
        raise ValueError("Namespaces list contains duplicates")
    return namespaces

def remap_airsim_image(input_image_ns, output_image_ns):
    return [
        (f"{input_image_ns}", f"{output_image_ns}/image_raw"),
        (f"{input_image_ns}/camera_info", f"{output_image_ns}/camera_info"),
        (f"{input_image_ns}/compressed", f"{output_image_ns}/image_raw/compressed"),
        (f"{input_image_ns}/compressedDepth", f"{output_image_ns}/image_raw/compressedDepth"),
        (f"{input_image_ns}/theora", f"{output_image_ns}/image_raw/theora")
    ]
