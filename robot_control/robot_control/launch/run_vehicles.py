#!/usr/bin/env python3
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os
from enum import IntEnum, auto
import yaml
from glob import glob

from airsim_utils.generate_settings import create_settings, DEFAULT_PAWN_BP
from airsim_utils.generate_settings import VehicleType as AirSimVehicleType
from airsim_utils.run_environment import run_environment
from ros2_utils.launch import get_local_arguments, parse_model_file, combine_names
from robot_control.launch.ignition import setup as setup_ignition
from robot_control.launch.gazebo import setup as setup_gazebo


# Get relative package directories
robot_control = get_package_share_directory("robot_control")


# API's and the simulators they work with
API_PAIRS = {
    "mavros": ["airsim", "gazebo", "ignition", "none"],
    "none": ["airsim", "ignition"]
}


class VehicleType(IntEnum):
    DRONE = 0
    ROVER = auto()
    PLANE = auto()


class SimType(IntEnum):
    NONE = 0
    GAZEBO = auto()
    AIRSIM = auto()
    IGNITION = auto()


class ApiType(IntEnum):
    NONE=0
    MAVROS=auto()


class SimSource(IntEnum):
    DEFAULT=0
    ROBOT=auto()
    NUAV=auto()


# Arguments with relevant info, type defaults to string
LAUNCH_ARGS = [
    {"name": "gui",             "default": "true",              "description": "Starts gazebo gui"},
    {"name": "verbose",         "default": "false",             "description": "Starts gazebo server with verbose outputs"},
    {"name": "gz_world",        "default": "empty.world",       "description": "Gazebo world to load"},
    {"name": "ign_world",       "default": "empty.sdf",         "description": "Ignition world to load"},
    {"name": "environment",     "default": "",                  "description": "Path to executable for running AirSim environment."},
    {"name": "pawn_bp",         "default": DEFAULT_PAWN_BP,     "description": "Pawn blueprint to spawn in AirSim."},
    {"name": "nb",              "default": "1",                 "description": "Number of vehicles to spawn.",
        "type": "int"},
    {"name": "namespaces",      "default": "[]",                "description": "List of namespaces. Will autofill if number spawned is greater than number of items. Ex: ['drone','rover'].",
        "type": "list"},
    {"name": "base_name",       "default": "",                  "description": "Prefix for all vehicles."},
    {"name": "log_level",       "default": "debug",             "description": "Sets log level of ros nodes."},
    {"name": "hitl",            "default": "false",             "description": "Flag to enable HITL.",
        "type": "bool"},
    {"name": "sim",             "default": "gazebo",            "description": "Simulation to use.",
        "choices": [e.name.lower() for e in SimType]},
    {"name": "vehicle_type",    "default": "drone",             "description": "Type of vehicle to spawn.",
        "choices": [e.name.lower() for e in VehicleType]},
    {"name": "api",             "default": "mavros",            "description": "API to use.",
        "choices": [e.name.lower() for e in ApiType]},
    {"name": "sim_source",    "default": "default",           "description": "Source for simulation.",
        "choices": [e.name.lower() for e in SimSource]},
    {"name": "model_name",    "default": "",                  "description": "Model to spawn in ignition or gazebo. Overrides sim_source."},
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
    sim_source = SimSource[args["sim_source"].upper()]

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
    namespaces = extract_namespaces(args)

    ld.append(
        LogInfo(msg=[
            f'Spawning vehicles with namespaces "{", ".join(namespaces)}".'
        ])
    )

    # Start simulator
    if sim == SimType.GAZEBO:
        setup_gazebo()
        ld.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [robot_control, os.path.sep, 'launch',
                        os.path.sep, 'gazebo.launch.py']
                ),
                launch_arguments={
                    'verbose':args['verbose'],
                    'gui': args['gui'],
                    'world': args['gz_world']
                }.items(),
            )
        )
        # Spawn Vehicles
        for i, namespace in enumerate(namespaces):
            # TODO: Figure out how to use multiple vehicles with HITL?
            ld += spawn_gz_vehicle(namespace=namespace, instance=i, mavlink_tcp_port=4560+i,
                                   mavlink_udp_port=14560+i, hil_mode=args["hitl"], vehicle_type=vehicle_type,
                                   model_source=sim_source, model_name=args["model_name"])
    elif sim == SimType.IGNITION:
        setup_ignition()
        ld.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [robot_control, os.path.sep, 'launch',
                        os.path.sep, 'ignition.launch.py']
                ),
                launch_arguments={
                    'verbose':args['verbose'],
                    'gui': args['gui'],
                    'world': args['ign_world']
                }.items(),
            )
        )
        # Spawn Vehicles
        for i, namespace in enumerate(namespaces):
            # TODO: Figure out how to use multiple vehicles with HITL?
            ld += spawn_ign_vehicle(namespace=namespace, instance=i, mavlink_tcp_port=4560+i,
                                    mavlink_udp_port=14560+i, hil_mode=args["hitl"], vehicle_type=vehicle_type,
                                    api=api, model_source=sim_source)
    elif sim == SimType.AIRSIM:
        ld += generate_airsim(args["hitl"], args["nb"], args["pawn_bp"], namespaces, args["environment"], log_level)
        ld.append(
            Node(
                package='airsim_ros', executable="airsim_node",
                output='screen',
                arguments=[
                    "--ros-args", "--log-level", f"airsim_ros_wrapper:={log_level}"
                ],
            ),
        )

    # Create executable call
    if api == ApiType.NONE:
        api_exe = sim.name.lower()
    else:
        api_exe = api.name.lower()
    vehicle_exe = f"{api_exe}_{vehicle_exe}"

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
                        # os.path.join(robot_control, "config", "px4_sim_params.yaml"),
                        # os.path.join(robot_control, "config", "px4_config.yaml"),
                        # os.path.join(robot_control, "config", "px4_pluginlists.yaml")
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
                     hil_mode=False, serial_device="/dev/ttyACM0", vehicle_type=VehicleType.DRONE,
                     model_source=SimSource.DEFAULT, model_name=""):
    """Spawns vehicle in running gazebo world.

    Args:
        namespace (str, optional): ROS namespace for all nodes. Defaults to "drone_0".
        instance (int, optional): Instance of PX4 SITL to start. Defaults to 0.
        mavlink_tcp_port (int, optional): TCP port to use with mavlink. Defaults to 4560.
        mavlink_udp_port (int, optional): UDP port to use with mavlink. Defaults to 14560.
        hil_mode (bool, optional): Flag that turns on HITL mode. Defaults to False.
        serial_device (str, optional): Path to PX4 serial device port. Defaults to "/dev/ttyACM0".
        model_source (SimSource, optional): Source for models to be derived from. Defaults to SimSource.DEFAULT.
        model_name (str, optional): Name of model to find in `GAZEBO_MODEL_PATH`. If empty uses model_source to find model. Defaults to "".
    """
    ld = []
    # TODO: URDF still needs to be implemented
    # Creates tmp URDF file with frog v2
    mappings = {"namespace": namespace,
                "mavlink_tcp_port": mavlink_tcp_port,
                "mavlink_udp_port": mavlink_udp_port,
                "serial_enabled": "1" if hil_mode else "0",
                "serial_device": serial_device,
                "serial_baudrate": "921600",
                "hil_mode": "1" if hil_mode else "0"}
    file_path = get_model_path("GAZEBO_MODEL_PATH", model_name)
    if file_path == "" and model_name == "":
        if model_source == SimSource.NUAV:
            pkg_nuav_gazebo = get_package_share_directory("nuav_gazebo")
            if vehicle_type == VehicleType.DRONE:
                mappings["camera_tilt"] = 0.0
                mappings["include_camera"] = True
                file_path = f'{pkg_nuav_gazebo}/models/robots/frog_v2/model.sdf.jinja'
            elif vehicle_type == VehicleType.ROVER:
                file_path = f'{pkg_nuav_gazebo}/models/robots/bullfrog/model.sdf.jinja'
            elif vehicle_type == VehicleType.PLANE:
                file_path = f'{pkg_nuav_gazebo}/models/robots/wallace/model.sdf.jinja'
            # TODO: add carrier?
            else:
                raise Exception(f"Vehicle type {vehicle_type.name} not supported")
        elif model_source == SimSource.ROBOT:
            pkg_robot_gazebo = get_package_share_directory("robot_gazebo")
            if vehicle_type == VehicleType.DRONE:
                file_path = f'{pkg_robot_gazebo}/models/iris_realsense/model.sdf.jinja'
            else:
                raise Exception(f"Vehicle type {vehicle_type.name} not supported")
        elif model_source == SimSource.DEFAULT:
            if vehicle_type == VehicleType.DRONE:
                file_path = f'{os.environ["PX4_AUTOPILOT"]}/Tools/sitl_gazebo/models/iris/iris.sdf.jinja'
            elif vehicle_type == VehicleType.ROVER:
                file_path = f'{os.environ["PX4_AUTOPILOT"]}/Tools/sitl_gazebo/models/r1_rover/r1_rover.sdf.jinja'
            elif vehicle_type == VehicleType.PLANE:
                file_path = f'{os.environ["PX4_AUTOPILOT"]}/Tools/sitl_gazebo/models/plane/plane.sdf.jinja'
            else:
                raise Exception(f"Vehicle type {vehicle_type.name} not supported")
        else:
            raise Exception(f"Model source {model_source.name} not supported")
    elif file_path == "" and model_name != "":
        raise Exception(f"Model {model_name} could not be found for gazebo")

    ld.append(
        LogInfo(msg=[f"Spawning model: {file_path}"])
    )
    tmp_path, robot_desc = parse_model_file(file_path, mappings)

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


def spawn_ign_vehicle(namespace="drone_0", instance=0, mavlink_tcp_port=4560, mavlink_udp_port=14560,
                      hil_mode=False, serial_device="/dev/ttyACM0", vehicle_type=VehicleType.DRONE,
                      api=ApiType.MAVROS, model_source=SimSource.DEFAULT, model_name=""):
    """Spawns vehicle in running gazebo world.

    Args:
        namespace (str, optional): ROS namespace for all nodes. Defaults to "drone_0".
        instance (int, optional): Instance of PX4 SITL to start. Defaults to 0.
        mavlink_tcp_port (int, optional): TCP port to use with mavlink. Defaults to 4560.
        mavlink_udp_port (int, optional): UDP port to use with mavlink. Defaults to 14560.
        hil_mode (bool, optional): Flag that turns on HITL mode. Defaults to False.
        serial_device (str, optional): Path to PX4 serial device port. Defaults to "/dev/ttyACM0".
    """
    ld = []
    # TODO: URDF still needs to be implemented
    # Creates tmp URDF file with frog v2
    mappings = {"namespace": namespace,
                "mavlink_tcp_port": mavlink_tcp_port,
                "mavlink_udp_port": mavlink_udp_port,
                "serial_enabled": "1" if hil_mode else "0",
                "serial_device": serial_device,
                "serial_baudrate": "921600",
                "hil_mode": "1" if hil_mode else "0"}
    file_path = get_model_path("IGN_GAZEBO_RESOURCE_PATH", model_name)
    if file_path == "" and model_name == "":
        if model_source == SimSource.ROBOT:
            pkg_robot_ignition = get_package_share_directory("robot_ignition")
            if vehicle_type == VehicleType.DRONE:
                if api == ApiType.MAVROS:
                    file_path = f'{pkg_robot_ignition}/models/x3_mavlink/model.sdf.jinja'
                    # file_path = f'{os.environ["PX4_AUTOPILOT"]}/Tools/simulation-ignition/models/x3/model.sdf'
                    # file_path = f'{os.environ["HOME"]}/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models/construction cone/2/model.sdf'
                elif api == ApiType.NONE:
                    file_path = f'{pkg_robot_ignition}/models/x3_ignition/model.sdf.jinja'
                    ld.append(
                        Node(
                            package="ros_ign_bridge", executable="parameter_bridge",
                            arguments=[
                                # World info
                                f"/world/empty/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                                # Multicopter control
                                f"/{namespace}/gazebo/command/twist@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                                f"/{namespace}/enable@std_msgs/msg/Bool]ignition.msgs.Boolean",
                                # Joint state publisher
                                f"/world/empty/model/{namespace}/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
                                # Odometry publisher
                                f"/model/{namespace}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                            ],
                            remappings=[
                                # World info
                                # (f"/world/empty/pose/info"),
                                # Multicopter control
                                (f"/{namespace}/gazebo/command/twist", f"/{namespace}/_ign/gazebo/command/twist"),
                                (f"/{namespace}/enable", f"/{namespace}/_ign/enable"),
                                # Joint state publisher
                                (f"/world/empty/model/{namespace}/joint_state", f"/{namespace}/_ign/joint_state"),
                                # Odometry publisher
                                (f"/model/{namespace}/odom", f"/{namespace}/_ign/odom"),
                            ],
                            output="screen"
                        )
                    )
                else:
                    raise Exception(f"Api '{api.name}' not supported for ignition drone")
            else:
                raise Exception(f"Vehicle type {vehicle_type.name} not supported")
        else:
            raise Exception(f"Model source {model_source.name} not supported")
    elif file_path == "" and model_name != "":
        raise Exception(f"Model {model_name} could not be found for ignition")

    ld.append(
        LogInfo(msg=[f"Spawning model: {file_path}"])
    )
    tmp_path, robot_desc = parse_model_file(file_path, mappings)

    # Spawns vehicle model using SDF or URDF file
    ld.append(
        Node(
            package="ros_ign_gazebo", executable="create",
            arguments=[
                "-file", tmp_path,
                "-name", namespace,    
                "-x", str(instance*2), "-y", str(0), "-z", str(0.15),
                "-R", str(0.0), "-P", str(0.0), "-Y", str(0.0)
            ],
            output="screen"
        )
    )
    return ld


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
    namespaces = args["namespaces"]
    len_ns = len(namespaces)
    if args["nb"] > len_ns:
        gen_num = args["nb"] - len_ns  # amount to generate
        for i in range(gen_num):
            namespaces.append(f"{args['base_name']}_{len_ns+i}")
    if len(namespaces) != len(set(namespaces)):
        raise ValueError("Namespaces list contains duplicates")
    return namespaces


def get_model_path(env: str, name: str):
    path = ""
    if name != "":
        gz_models_paths = os.environ[env]
        gz_models_paths = gz_models_paths.replace("::",":").split(":")
        print(gz_models_paths)
        for gz_path in gz_models_paths:
            model_path = glob(os.path.join(gz_path,name,"*.sdf.*"))
            if model_path:
                return model_path[0]
    return ""
