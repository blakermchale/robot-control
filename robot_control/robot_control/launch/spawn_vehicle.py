#!/usr/bin/env python3
from launch_ros.actions import Node
from launch.actions import LogInfo

from ament_index_python.packages import get_package_share_directory
import os
from glob import glob

from ros2_utils.launch import get_local_arguments, parse_model_file, combine_names
from robot_control.launch.ignition import setup as setup_ignition
from robot_control.launch.gazebo import setup as setup_gazebo
from robot_control.launch.structs import SimType, VehicleType, ApiType, ROBOT_CONTROL_PKG, SimSource


# API's and the simulators they work with
API_PAIRS = {
    "mavros": ["airsim", "gazebo", "ignition", "none"],
    "inherent": ["airsim", "ignition"],
    "none": ["airsim", "gazebo", "ignition", "none"],
}

DEFAULT_VEHICLE_YAML = os.path.join(ROBOT_CONTROL_PKG,"config/vehicle_params.yaml")
# Arguments with relevant info, type defaults to string
LAUNCH_ARGS = [
    # BASE INPUTS
    {"name": "namespace",       "default": "",                   "description": "Vehicle namespace. Will autofill if not given. Ex: drone_0."},
    {"name": "log_level",       "default": "debug",             "description": "Sets log level of ros nodes."},
    {"name": "sim",             "default": "gazebo",            "description": "Simulation being used.",
        "choices": [e.name.lower() for e in SimType]},
    {"name": "vehicle_type",    "default": "drone",             "description": "Type of vehicle to spawn.",
        "choices": [e.name.lower() for e in VehicleType]},
    {"name": "model_name",    "default": "",                    "description": "Model to spawn in ignition or gazebo. Overrides sim_source."},
    {"name": "vehicle_yaml",  "default": DEFAULT_VEHICLE_YAML,  "description": "YAML file to load vehicle parameters from."},
    {"name": "camera",        "default": "false",               "description": "Flag indicating whether to use a camera."},
    {"name": "api",             "default": "mavros",            "description": "API to use.",
        "choices": [e.name.lower() for e in ApiType]},
    # MAVROS
    {"name": "hitl",            "default": "false",             "description": "Flag to enable HITL.",
        "type": "bool"},
    {"name": "instance",        "default": "1",                   "description": "Vehicle instance. Used for calculating mavros variables. Ex: 0.",
        "type": "int"},
    # CONFIG
    {"name": "sim_source",    "default": "default",             "description": "Source for simulation.",
        "choices": [e.name.lower() for e in SimSource]},
]


def launch_setup(context, *largs, **kwargs):
    """Allows declaration of launch arguments within the ROS2 context
    """
    largs = get_local_arguments(LAUNCH_ARGS, context)

    # Check for empty variables
    if not os.environ.get("PX4_AUTOPILOT"):
        raise Exception("PX4_AUTOPILOT env variable must be set")

    ld = []

    if largs["api"] not in API_PAIRS.keys():
        raise Exception(f"API '{largs['api']}' must be specified in API_PAIRS")
    if largs["sim"] not in API_PAIRS[largs["api"]]:
        raise Exception(f"API '{largs['api']}' does not work with simulation '{largs['sim']}'")

    # Choose base name according to vehicle type
    vehicle_type = VehicleType[largs['vehicle_type'].upper()]
    sim = SimType[largs['sim'].upper()]
    api = ApiType[largs["api"].upper()]
    sim_source = SimSource[largs["sim_source"].upper()]

    namespace = largs["namespace"]
    i = largs["instance"]
    main_msg = f'Spawning {largs["vehicle_type"]} \'{largs["namespace"]}\' - '
    main_parts = []
    main_parts.append(f'api: {largs["api"]}')
    main_parts.append(f'sim: {largs["sim"]}')
    main_parts.append(f'log_level: {largs["log_level"]}')
    if largs["model_name"] != "":
        main_parts.append(f'model: {largs["model_name"]}')
    else:
        main_parts.append(f'sim_source: {largs["model_name"]}')
    if largs["hitl"]: main_parts.append(f'hitl')
    if largs["camera"]: main_parts.append(f'camera')
    main_parts.append(f'api: {largs["api"]}')
    if api == ApiType.MAVROS: main_parts.append(f'instance: {largs["instance"]}')
    main_msg += ', '.join(main_parts)
    ld.append(LogInfo(msg=[main_msg]))

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
    log_level = largs["log_level"].upper()

    # Start simulator
    if sim == SimType.GAZEBO:
        setup_gazebo()
        # TODO: Figure out how to use multiple vehicles with HITL?
        ld += spawn_gz_vehicle(namespace=namespace, instance=i, mavlink_tcp_port=4560+i,
                                mavlink_udp_port=14560+i, hil_mode=largs["hitl"], vehicle_type=vehicle_type,
                                model_source=sim_source, model_name=largs["model_name"])
    elif sim == SimType.IGNITION:
        setup_ignition()
        # TODO: Figure out how to use multiple vehicles with HITL?
        ld += spawn_ign_vehicle(namespace=namespace, instance=i, mavlink_tcp_port=4560+i,
                                mavlink_udp_port=14560+i, hil_mode=largs["hitl"], vehicle_type=vehicle_type,
                                api=api, model_source=sim_source, model_name=largs["model_name"])
    elif sim == SimType.AIRSIM:
        ld.append(
            LogInfo(msg=[f"AirSim should be started from outside launch, assuming it is..."])
        )

    # Create executable call
    if api == ApiType.INHERENT:
        api_exe = sim.name.lower()
    else:
        api_exe = api.name.lower()
    vehicle_exe = f"{api_exe}_{vehicle_exe}"

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
    elif api == ApiType.INHERENT:
        pass
    elif api == ApiType.NONE:
        return ld
    else:
        raise Exception(f"API {api.name} not supported yet")
    # Launch vehicle executable that creates common ROS2 API
    node_name = combine_names([namespace, "vehicle"], ".")
    ld.append(
        Node(
            package='robot_control', executable=vehicle_exe,
            output='screen',
            namespace=namespace,
            emulate_tty=True,
            arguments=[
                "--instance", str(i),
                "--ros-args", "--log-level", f"{node_name}:={log_level}"
            ],
            parameters=[
                largs["vehicle_yaml"],
            ]
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
        LogInfo(msg=[f"Using model: {file_path}"])
    )
    tmp_path, robot_desc = parse_model_file(file_path, mappings)

    # # https://github.com/ros/sdformat_urdf 
    # ld.append(
    #     Node(
    #         package='robot_state_publisher', executable='robot_state_publisher',
    #         output='screen',
    #         namespace=namespace,
    #         parameters=[{
    #             'robot_description': robot_desc if robot_desc is None else tmp_path,
    #             'ignore_timestamp': True
    #         }]
    #     )
    # )

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
                elif api == ApiType.INHERENT:
                    file_path = f'{pkg_robot_ignition}/models/x3_ignition/model.sdf.jinja'
                else:
                    raise Exception(f"Api '{api.name}' not supported for ignition drone")
            else:
                raise Exception(f"Vehicle type {vehicle_type.name} not supported")
        else:
            raise Exception(f"Model source {model_source.name} not supported")
    elif file_path == "" and model_name != "":
        raise Exception(f"Model {model_name} could not be found for ignition")
    
    # Exposes ignition topics to ros
    bridge_args = [
        # Realsense Camera Info
        f"/world/runway/model/{namespace}/link/realsense_d435/sensor/realsense_d435/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        # Realsense Depth Camera  TODO: don't hardcode world runway name in ign topic
        f"/world/runway/model/{namespace}/link/realsense_d435/sensor/realsense_d435/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
        # Realsense Color Camera
        f"/world/runway/model/{namespace}/link/realsense_d435/sensor/realsense_d435/image@sensor_msgs/msg/Image[ignition.msgs.Image",
        # Realsense Point Cloud
        f"/world/runway/model/{namespace}/link/realsense_d435/sensor/realsense_d435/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
    ]
    bridge_remaps = [
        # Realsense Depth Camera
        (f"/world/runway/model/{namespace}/link/realsense_d435/sensor/realsense_d435/depth_image", f"/{namespace}/realsense/aligned_depth_to_color/image_raw"),
        # (f"", f"/{namespace}/realsense/aligned_depth_to_color/image_raw/compressed"),
        (f"/world/runway/model/{namespace}/link/realsense_d435/sensor/realsense_d435/camera_info", f"/{namespace}/realsense/aligned_depth_to_color/camera_info"),
        # Realsense Color Camera
        (f"/world/runway/model/{namespace}/link/realsense_d435/sensor/realsense_d435/image", f"/{namespace}/realsense/color/image_raw"),
        # (f"", f"/{namespace}/realsense/color/image_raw/compressed"),
        # (f"", f"/{namespace}/realsense/color/camera_info")
        # Realsense Point Cloud
        (f"/world/runway/model/{namespace}/link/realsense_d435/sensor/realsense_d435/points", f"/{namespace}/realsense/depth/color/points")
    ]
    if api == ApiType.INHERENT:
        bridge_args += [
            # World info
            f"/world/empty/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            # Multicopter control
            f"/{namespace}/gazebo/command/twist@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            f"/{namespace}/enable@std_msgs/msg/Bool]ignition.msgs.Boolean",
            # Joint state publisher
            f"/world/empty/model/{namespace}/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
            # Odometry publisher
            f"/model/{namespace}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        ]
        bridge_remaps += [
            # World info
            # (f"/world/empty/pose/info"),
            # Multicopter control
            (f"/{namespace}/gazebo/command/twist", f"/{namespace}/_ign/gazebo/command/twist"),
            (f"/{namespace}/enable", f"/{namespace}/_ign/enable"),
            # Joint state publisher
            (f"/world/empty/model/{namespace}/joint_state", f"/{namespace}/_ign/joint_state"),
            # Odometry publisher
            (f"/model/{namespace}/odom", f"/{namespace}/_ign/odom"),
        ]
    ld += [
        Node(
            package="ros_ign_bridge", executable="parameter_bridge",
            arguments=bridge_args,
            remappings=bridge_remaps,
            output="screen"
        ),
        Node(package="ros2_utils", executable="relay", namespace=namespace,
            arguments=[f"realsense/aligned_depth_to_color/camera_info", f"realsense/color/camera_info"]
        )
    ]

    ld.append(
        LogInfo(msg=[f"Using model: {file_path}"])
    )
    tmp_path, robot_desc = parse_model_file(file_path, mappings)

    # Spawns vehicle model using SDF or URDF file
    ld.append(
        Node(
            package="ros_ign_gazebo", executable="create",
            arguments=[
                "-file", tmp_path,
                "-name", namespace,    
                "-x", str(0), "-y", str(instance*2), "-z", str(0.15),
                "-R", str(0.0), "-P", str(0.0), "-Y", str(0.0)
            ],
            output="screen"
        )
    )
    return ld


def get_model_path(env: str, name: str):
    path = ""
    if name != "":
        gz_models_paths = os.environ[env]
        gz_models_paths = gz_models_paths.replace("::",":").split(":")
        for gz_path in gz_models_paths:
            model_path = glob(os.path.join(gz_path,name,"*.sdf.*"))
            if model_path:
                return model_path[0]
    return ""