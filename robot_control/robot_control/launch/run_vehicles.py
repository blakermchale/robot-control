#!/usr/bin/env python3
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
import yaml

from airsim_utils.generate_settings import create_settings, DEFAULT_PAWN_BP
from airsim_utils.generate_settings import VehicleType as AirSimVehicleType
from airsim_utils.run_environment import run_environment
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
    {"name": "environment",     "default": "",                  "description": "Path to executable for running AirSim environment."},
    {"name": "pawn_bp",         "default": DEFAULT_PAWN_BP,     "description": "Pawn blueprint to spawn in AirSim."},
]
# Remove launch args that cannot be generically used
ENV_LAUNCH_ARGS = [x for x in ENV_LAUNCH_ARGS if not x["name"] in ["log_level"]]
SPAWN_LAUNCH_ARGS = [x for x in SPAWN_LAUNCH_ARGS if not x["name"] in ["namespace", "instance", "log_level", "sim"]]
LAUNCH_ARGS += ENV_LAUNCH_ARGS
LAUNCH_ARGS += SPAWN_LAUNCH_ARGS


def launch_setup(context, *largs, **kwargs):
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

    # Start up environment
    env_args = [(a["name"], str(largs[a["name"]])) for a in ENV_LAUNCH_ARGS]
    ld.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ROBOT_CONTROL_PKG, os.path.sep, 'launch',
                    os.path.sep, 'environment.launch.py']
            ),
            launch_arguments=env_args,
        )
    )

    # Spawn vehicles
    team_yaml = largs["team_yaml"]
    # NOTE: example team_yaml yaml file in `config/team.yaml`
    if team_yaml != "":
        ld.append(
            LogInfo(msg=[
                f'Using `team_yaml` file "{team_yaml}" to load vehicles.'
            ])
        )
    team = get_team(largs, context)
    ld.append(
        LogInfo(msg=[
            f'Spawning vehicles with namespaces "{", ".join(team.keys())}".'
        ])
    )
    for spawn_args in team.values():
        ld += add_spawn_launch(spawn_args)
        

    if sim == SimType.AIRSIM:
        namespaces = extract_namespaces(largs)
        ld += generate_airsim(largs["hitl"], largs["nb"], largs["pawn_bp"], namespaces, largs["environment"], log_level)
        ld.append(
            Node(
                package='airsim_ros', executable="airsim_node",
                output='screen',
                arguments=[
                    "--ros-args", "--log-level", f"airsim_ros_wrapper:={log_level}"
                ],
            ),
        )

    return ld


def add_spawn_launch(spawn_args):
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ROBOT_CONTROL_PKG, os.path.sep, 'launch',
                    os.path.sep, 'spawn_vehicle.launch.py']
            ),
            launch_arguments=spawn_args.items(),
        )
    ]


def get_team(largs, context):
    """Gets a dictionary of all vehicles being spawned and their fields."""
    default_spawn_args = {a["name"]: str(largs[a["name"]]) for a in SPAWN_LAUNCH_ARGS}
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
                if name in spawn_args:
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


def generate_airsim(hitl=False, nb=1, pawn_bp=DEFAULT_PAWN_BP, namespaces=[], environment="", log_level="DEBUG"):
    ld = []
    # Generate settings
    if hitl:
        # FIXME: issue with using custom LLA during HITL, just use default
        create_settings(nb=nb, pawn_bp=pawn_bp,
                        hitl=hitl)
    else:
        create_settings(nb=nb, pawn_bp=pawn_bp,
                        lat=42.3097, lon=-71.0959, alt=141.0, hitl=hitl,
                        vehicle_type=AirSimVehicleType.SIMPLEFLIGHT,
                        namespaces=namespaces)
        os.environ["PX4_SIM_HOST_ADDR"] = os.environ["WSL_HOST_IP"]
    if not environment:
        raise Exception(f"AirSim environment must be valid not '{environment}'")
    run_env = run_environment(env=environment)
    # FIXME: resolve run_env flag always being false
    # if not run_env:
    #     ld.append(
    #         LogInfo(msg=[
    #             'Exiting early because run environment failed.'
    #         ])
    #     )
    #     return ld
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
