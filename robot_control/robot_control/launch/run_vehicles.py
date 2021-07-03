#!/usr/bin/env python3
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os
import jinja2
import xacro
from enum import IntEnum
import yaml

from airsim_utils.generate_settings import create_settings, DEFAULT_PAWN_BP
from airsim_utils.generate_settings import VehicleType as AirSimVehicleType
from airsim_utils.run_environment import run_environment
from robot_control.launch.common import get_local_arguments


# Get relative package directories
robot_control = get_package_share_directory("robot_control")


# API's and the simulators they work with
API_PAIRS = {
    "mavros": ["airsim", "gazebo", "none"],
    "none": ["airsim"]
}


class VehicleType(IntEnum):
    DRONE = 0
    ROVER = 1
    PLANE = 2


class SimType(IntEnum):
    NONE = 0
    GAZEBO = 1
    AIRSIM = 2


class ApiType(IntEnum):
    NONE = 0
    MAVROS = 1


# Arguments with relevant info, type defaults to string
LAUNCH_ARGS = [
    {"name": "gui",             "default": "true",              "description": "Starts gazebo gui"},
    {"name": "verbose",         "default": "false",             "description": "Starts gazebo server with verbose outputs"},
    {"name": "world",           "default": "empty.world",       "description": "Gazebo world to load"},
    {"name": "environment",     "default": "",                  "description": "Path to executable for running AirSim environment."},
    {"name": "pawn_bp",         "default": DEFAULT_PAWN_BP,     "description": "Pawn blueprint to spawn in AirSim."},
    {"name": "nb",              "default": "1",                 "description": "Number of vehicles to spawn.",
        "type": "int"},
    {"name": "namespaces",      "default": "[]",                "description": "List of namespaces. Will autofill if number spawned is greater than number of items. Ex: ['drone','rover'].",
        "type": "list"},
    {"name": "base_name",       "default": "",                  "description": "Prefix for all vehicles."},
    {"name": "log_level",       "default": "debug",             "description": "Sets log level of ros nodes."},
    {"name": "sim",             "default": "gazebo",            "description": "Simulation to use.",
        "choices": [e.name.lower() for e in SimType]},
    {"name": "vehicle_type",    "default": "drone",             "description": "Type of vehicle to spawn.",
        "choices": [e.name.lower() for e in VehicleType]},
    {"name": "api",             "default": "mavros",            "description": "API to use.",
        "choices": [e.name.lower() for e in ApiType]},
    {"name": "hitl",            "default": "false",             "description": "Flag to enable HITL.",
        "type": "bool"}
]


def launch_setup(context, *args, **kwargs):
    """Allows declaration of launch arguments within the ROS2 context
    """
    args = get_local_arguments(LAUNCH_ARGS, context)

    # Check for empty variables
    if not os.environ.get("PX4_AUTOPILOT"):
        raise Exception("PX4_AUTOPILOT env variable must be set")

    ld = []

    # Check for config file
    if os.environ.get("ROBOT_CONTROL_CONFIG"):
        ld.append(
            LogInfo(msg=[
                'Launching using `ROBOT_CONTROL_CONFIG` file instead of passed in arguments.'
            ])
        )
        param_file = ParameterFile(
            param_file=os.environ["ROBOT_CONTROL_CONFIG"],
            allow_substs=True)
        param_file_path = param_file.evaluate(context)
        with open(param_file_path, 'r') as f:
            config_vals = yaml.load(f, Loader=yaml.FullLoader)
        # Overrides passed launch args with config files
        for k, v in config_vals.items():
            args[k] = v  #TODO: make sure this is the proper type

    if args["api"] not in API_PAIRS.keys():
        raise Exception(f"API '{args['api']}' must be specified in API_PAIRS")
    if args["sim"] not in API_PAIRS[args["api"]]:
        raise Exception(f"API '{args['api']}' does not work with simulation '{args['sim']}'")

    # Choose base name according to vehicle type
    if args['base_name'] == "":
        args["base_name"] = args['vehicle_type']
    vehicle_type = VehicleType[args['vehicle_type'].upper()]
    sim = SimType[args['sim'].upper()]
    api = ApiType[args["api"].upper()]

    if vehicle_type == VehicleType.DRONE:
        model = "iris"
        vehicle_exe = "drone"
    elif vehicle_type == VehicleType.ROVER:
        model = "r1_rover"
        vehicle_exe = "rover"
        raise Exception("Rover not supported yet")
    elif vehicle_type == VehicleType.PLANE:
        model = "plane"
        vehicle_exe = "plane"
        raise Exception("Plane not supported yet")
    else:
        raise Exception("Vehicle type needs to be specified")

    # Establish log level
    log_level = args["log_level"].upper()

    # Pre-process namespaces
    namespaces = args["namespaces"]
    len_ns = len(namespaces)
    if args["nb"] > len_ns:
        gen_num = args["nb"] - len_ns  # amount to generate
        for i in range(gen_num):
            namespaces.append(f"{args['base_name']}_{len_ns+i}")
    if len(namespaces) != len(set(namespaces)):
        raise ValueError("Namespaces list contains duplicates")

    # Start simulator
    if sim == SimType.GAZEBO:
        ld.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [robot_control, os.path.sep, 'launch',
                        os.path.sep, 'gazebo.launch.py']
                ),
                launch_arguments={
                    'verbose':args['verbose'],
                    'gui': args['gui'],
                    'world': args['world']
                }.items(),
            )
        )
        vehicle_exe = f"mavros_{vehicle_exe}"
        # Spawn Vehicles
        for i, namespace in enumerate(namespaces):
            # TODO: Figure out how to use multiple vehicles with HITL?
            ld += spawn_gz_vehicle(namespace=namespace, instance=i, mavlink_tcp_port=4560+i,
                                   mavlink_udp_port=14560+i, hil_mode=args["hitl"], vehicle_type=vehicle_type)
    elif sim == SimType.AIRSIM:
        # Generate settings
        if args["hitl"]:
            # FIXME: issue with using custom LLA during HITL, just use default
            create_settings(nb=args['nb'], pawn_bp=args["pawn_bp"],
                            hitl=args['hitl'])
        else:
            create_settings(nb=args['nb'], pawn_bp=args["pawn_bp"],
                            lat=42.3097, lon=-71.0959, alt=141.0, hitl=args["hitl"],
                            vehicle_type=AirSimVehicleType.SIMPLEFLIGHT,
                            namespaces=namespaces)
            os.environ["PX4_SIM_HOST_ADDR"] = os.environ["WSL_HOST_IP"]
        if not args["environment"]:
            raise Exception(f"AirSim environment must be valid not '{args['environment']}'")
        run_env = run_environment(env=args["environment"])
        # FIXME: resolve run_env flag always being false
        # if not run_env:
        #     ld.append(
        #         LogInfo(msg=[
        #             'Exiting early because run environment failed.'
        #         ])
        #     )
        #     return ld
        vehicle_exe = f"airsim_{vehicle_exe}"

    for i, namespace in enumerate(namespaces):
        if api == ApiType.MAVROS:
            build_path=f"{os.environ['PX4_AUTOPILOT']}/build/px4_sitl_default"
            ld.append(
                Node(
                    package='robot_control', executable="sitl",
                    output='screen',
                    namespace=namespace,
                    arguments=[
                        "--instance", str(i),
                        "--build-path", build_path,
                    ],
                ),
            )
            ld.append(
                Node(
                    package="mavros", executable="mavros_node",
                    output="screen",
                    namespace=f"{namespace}/mavros",
                    # FIXME: parameters sometimes cause crash
                    parameters=[{
                            "fcu_url": "udp://:14540@127.0.0.1:14557",
                            "gcs_url": "",
                            "target_system_id": 1,
                            "target_component_id": 1,
                            "fcu_protocol": "v2.0",
                        },
                        os.path.join(robot_control, "config", "px4_config.yaml"),
                        os.path.join(robot_control, "config", "px4_pluginlists.yaml")
                    ]
                )
            )
            # raise NotImplementedError("MAVROS api not implemented yet")
        elif api == ApiType.NONE:
            pass
        else:
            raise Exception(f"API {api.name} not supported yet")
        # Launch vehicle executable that creates common ROS2 API
        node_name = combine_names([namespace, "vehicle"], ".")
        ld.append(
            Node(
                package='robot_control', executable=vehicle_exe,
                output='screen',
                namespace=namespace,
                arguments=[
                    "--instance", str(i),
                    "--ros-args", "--log-level", f"{node_name}:={log_level}"
                ],
            ),
        )
    return ld


def spawn_gz_vehicle(namespace="drone_0", instance=0, mavlink_tcp_port=4560, mavlink_udp_port=14560,
                     hil_mode=False, serial_device="/dev/ttyACM0", vehicle_type=VehicleType.DRONE):
    """Spawns vehicle in running gazebo world.

    Args:
        namespace (str, optional): ROS namespace for all nodes. Defaults to "drone_0".
        instance (int, optional): Instance of PX4 SITL to start. Defaults to 0.
        mavlink_tcp_port (int, optional): TCP port to use with mavlink. Defaults to 4560.
        mavlink_udp_port (int, optional): UDP port to use with mavlink. Defaults to 14560.
        hil_mode (bool, optional): Flag that turns on HITL mode. Defaults to False.
        serial_device (str, optional): Path to PX4 serial device port. Defaults to "/dev/ttyACM0".
    """
    # TODO: URDF still needs to be implemented
    # Creates tmp URDF file with frog v2
    mappings = {"namespace": namespace,
                "mavlink_tcp_port": mavlink_tcp_port,
                "mavlink_udp_port": mavlink_udp_port,
                "serial_enabled": "1" if hil_mode else "0",
                "serial_device": serial_device,
                "serial_baudrate": "921600",
                "hil_mode": "1" if hil_mode else "0"}
    if vehicle_type == VehicleType.DRONE:
        file_path = f'{os.environ["PX4_AUTOPILOT"]}/Tools/sitl_gazebo/models/iris/iris.sdf.jinja'
    elif vehicle_type == VehicleType.ROVER:
        file_path = f'{os.environ["PX4_AUTOPILOT"]}/Tools/sitl_gazebo/models/r1_rover/r1_rover.sdf.jinja'
    elif vehicle_type == VehicleType.PLANE:
        file_path = f'{os.environ["PX4_AUTOPILOT"]}/Tools/sitl_gazebo/models/plane/plane.sdf.jinja'
    else:
        raise Exception("Vehicle type needs to be specified")

    robot_desc = None
    if file_path.split('.')[-1] == "xacro":
        tmp_path = f'/tmp/{namespace}.urdf'
        doc = xacro.process_file(file_path, mappings=mappings)
        robot_desc = doc.toprettyxml(indent='  ')
        tmp_file = open(tmp_path, 'w')
        tmp_file.write(robot_desc)
        tmp_file.close()
    elif file_path.split('.')[-1] == "erb":
        tmp_path = f'/tmp/{namespace}.sdf'
        cmd = "erb"
        for (key, val) in mappings.items():
            cmd += f" {key}={val}"
        cmd += f" {file_path} > {tmp_path}"
        os.system(cmd)
    elif file_path.split('.')[-1] == "jinja":
        tmp_path = f'/tmp/{namespace}.sdf'
        templateFilePath = jinja2.FileSystemLoader(os.path.dirname(file_path))
        jinja_env = jinja2.Environment(loader=templateFilePath)
        j_template = jinja_env.get_template(os.path.basename(file_path))
        output = j_template.render(mappings)
        with open(tmp_path, 'w') as sdf_file:
            sdf_file.write(output)
    else:
        tmp_path = file_path

    ld = []
    # Spawns vehicle model using SDF or URDF file
    ld.append(
        Node(
            package="gazebo_ros", executable="spawn_entity.py",
            arguments=[
                "-entity", namespace, "-file", tmp_path,
                "-robot_namespace", namespace,
                "-spawn_service_timeout", "120.0",
                "-x", str(instance*2), "-y", str(0), "-z", str(0.15),
                "-R", str(0.0), "-P", str(0.0), "-Y", str(0.0)
            ],
            output="screen"
        )
    )
    return ld


def combine_names(l: list, sep: str):
    l = list(filter(None, l))  # Remove empty strings
    return sep.join(l)
