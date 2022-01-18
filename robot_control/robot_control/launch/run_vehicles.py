#!/usr/bin/env python3
from robot_control.launch.structs import ApiType
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
import yaml
import numpy as np

from airsim_utils.generate_settings import get_empty_settings, add_vehicle_settings, write_settings
from airsim_utils.generate_settings import VehicleType as AirSimVehicleType
from ros2_utils.launch import get_local_arguments
from robot_control.launch.structs import ScenarioType, ROBOT_CONTROL_PKG, SimType
from robot_control.launch.environment import LAUNCH_ARGS as ENV_LAUNCH_ARGS
from robot_control.launch.spawn_vehicle import LAUNCH_ARGS as SPAWN_LAUNCH_ARGS


DEFAULT_SCENARIO = ScenarioType.NONE.name.lower()
# Arguments with relevant info, type defaults to string
LAUNCH_ARGS = [
    {"name": "team_yaml",     "default": "",                    "description": "Path to yaml file that contains namespaces for a vehicle and their relevant launch args. See examples/ folder."},
    {"name": "scenario",      "default": DEFAULT_SCENARIO,      "description": "Scenario for robot. Changes types of vehicles, sensors, and environment used.",
        "choices": [e.name.lower() for e in ScenarioType]},
    {"name": "nb",              "default": "1",                 "description": "Number of vehicles to spawn.",
        "type": "int"},
    {"name": "namespaces",      "default": "[]",                "description": "List of namespaces. Will autofill if number spawned is greater than number of items. Ex: ['drone','rover'].",
        "type": "list"},
    {"name": "base_name",       "default": "",                  "description": "Prefix for all vehicles."},
    {"name": "pawn_bp",         "default": "",     "description": "Pawn blueprint to spawn in AirSim."},
]
# Remove launch args that cannot be generically used
SPAWN_LAUNCH_ARGS = [x for x in SPAWN_LAUNCH_ARGS if not x["name"] in ["namespace", "instance", "log_level", "sim"]]
LAUNCH_ARGS += ENV_LAUNCH_ARGS
LAUNCH_ARGS += SPAWN_LAUNCH_ARGS


def launch_setup(context, *args, **kwargs):
    """Allows declaration of launch arguments within the ROS2 context
    """
    ld = []

    # Check for config file
    yaml_file = ""
    # NOTE: example ROBOT_CONTROL_CONFIG yaml file in `config/robot_control.yaml`
    if os.environ.get("ROBOT_CONTROL_CONFIG"):
        ld.append(
            LogInfo(msg=[
                'Launching using `ROBOT_CONTROL_CONFIG` file instead of passed in arguments.'
            ])
        )
        yaml_file = os.environ["ROBOT_CONTROL_CONFIG"]
    largs = get_local_arguments(LAUNCH_ARGS, context, yaml_file=yaml_file)        

    sim = SimType[largs['sim'].upper()]

    # Establish log level
    log_level = largs["log_level"].upper()

    team_yaml = largs["team_yaml"]
    # NOTE: example team_yaml yaml file in `config/team.yaml`
    if team_yaml != "":
        ld.append(
            LogInfo(msg=[
                f'Using `team_yaml` file "{team_yaml}" to load vehicles.'
            ])
        )
    team = get_team(largs, context)

    # AirSim needs to be started outside since it doesn't follow normal process of (start sim -> spawn vehicle)
    #  instead you must (specify vehicles -> start sim)
    if sim == SimType.AIRSIM:
        ld += generate_settings_from_team(team)

    # Start up environment
    env_args = [(a["name"], str(largs[a["name"]])) for a in ENV_LAUNCH_ARGS]
    env_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ROBOT_CONTROL_PKG, os.path.sep, 'launch',
                os.path.sep, 'environment.launch.py']
        ),
        launch_arguments=env_args,
    )
    ld.append(env_ld)

    if sim == SimType.AIRSIM:
        if not os.environ.get("WSL_HOST_IP"):
            raise ValueError("WSL_HOST_IP must be set for AirSim to connect")
        remapped_images = []
        for namespace in team.keys():
            remapped_images += remap_airsim_image(f"/{namespace}/realsense/Scene", f"/{namespace}/realsense/color")
            remapped_images += remap_airsim_image(f"/{namespace}/realsense/DepthPlanar", f"/{namespace}/realsense/aligned_depth_to_color")
        ld += [
            Node(
                package='airsim_ros', executable="airsim_node",
                output='screen',
                arguments=[
                    "--ros-args", "--log-level", f"airsim_ros_wrapper:={log_level}",
                    "-p", f"host_ip:={os.environ['WSL_HOST_IP']}",
                ],
                remappings=remapped_images
            ),
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
        ]

    # Spawn vehicles
    ld.append(
        LogInfo(msg=[
            f'Spawning vehicles with namespaces "{", ".join(team.keys())}".'
        ])
    )
    for spawn_args in team.values():
        ld += add_spawn_launch(spawn_args)

    return ld


def add_spawn_launch(spawn_args):
    # Need to make sure launch args are strings
    spawn_args_str = {k: v if isinstance(v, dict) else str(v) for k, v in spawn_args.items()}
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ROBOT_CONTROL_PKG, os.path.sep, 'launch',
                    os.path.sep, 'spawn_vehicle.launch.py']
            ),
            launch_arguments=spawn_args_str.items(),
        )
    ]


def get_team(largs, context, default_team_args=SPAWN_LAUNCH_ARGS):
    """Gets a dictionary of all vehicles being spawned and their fields."""
    default_spawn_args = {a["name"]: largs[a["name"]] for a in default_team_args}
    team_yaml = largs["team_yaml"]
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
        namespaces = extract_namespaces(largs)
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
    if args['base_name'] == "":
        args["base_name"] = args['vehicle_type']
    namespaces = args["namespaces"]
    len_ns = len(namespaces)
    if args["nb"] > len_ns:
        gen_num = args["nb"] - len_ns  # amount to generate
        for i in range(gen_num):
            namespaces.append(f"{args['base_name']}_{len_ns+i}")
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
