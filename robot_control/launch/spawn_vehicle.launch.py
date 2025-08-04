#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from ros2_utils.launch import get_launch_arguments
from ros2_utils.launch_manager import LaunchManager
from robot_control.launch.structs import SimType, VehicleType, ApiType, ROBOT_CONTROL_PKG, SimSource
import os
from ros2_utils.launch import parse_model_file, combine_names
from robot_control.launch.ignition import setup as setup_ignition
import numpy as np
from launch_ros.actions import Node
from glob import glob


# API's and the simulators they work with
API_PAIRS = {
    "mavros": ["airsim", "gazebo", "ignition", "none"],
    "inherent": ["airsim", "ignition"],
    "none": ["airsim", "gazebo", "ignition", "none"],
}

DEFAULT_VEHICLE_YAML = os.path.join(ROBOT_CONTROL_PKG,"config/vehicle_params.yaml")


def launch_setup(args):
    lm = LaunchManager()

    # Check for empty variables
    if not os.environ.get("PX4_AUTOPILOT"):
        raise Exception("PX4_AUTOPILOT env variable must be set")

    if args.api not in API_PAIRS.keys():
        raise Exception(f"API '{args.api}' must be specified in API_PAIRS")
    if args.sim not in API_PAIRS[args.api]:
        raise Exception(f"API '{args.api}' does not work with simulation '{args.sim}'")

    # Choose base name according to vehicle type
    vehicle_type = VehicleType[args.vehicle_type.upper()]
    sim = SimType[args.sim.upper()]
    api = ApiType[args.api.upper()]
    sim_source = SimSource[args.sim_source.upper()]

    namespace = args.namespace
    i = args.instance
    if sim != SimType.AIRSIM:
        lm.add_launch_description(get_main_msg(args))

    if vehicle_type == VehicleType.DRONE:
        model = "iris"
        vehicle_exe = "drone"
    elif vehicle_type == VehicleType.ROVER:
        model = "r1_rover"
        vehicle_exe = "rover"
    elif vehicle_type == VehicleType.PLANE:
        model = "plane"
        vehicle_exe = "plane"
        raise Exception("Plane not supported yet")
    else:
        raise Exception("Vehicle type needs to be specified")
    

    # Establish log level
    log_level = args.log_level.upper()

    # Spawn vehicle in sim
    x, y, z, roll, pitch, yaw = args.x, args.y, args.z, args.roll, args.pitch, args.yaw
    roll = np.deg2rad(roll)
    pitch = np.deg2rad(pitch)
    yaw = np.deg2rad(yaw)
    if np.isnan(x): x = 0.0
    if np.isnan(y): y = int(i)*2
    if np.isnan(z): z = 0.15
    if sim == SimType.GAZEBO:
        raise NotImplementedError
        # setup_gazebo()
        # ld += spawn_gz_vehicle(namespace=namespace, instance=i, mavlink_tcp_port=4560+i,
        #                         mavlink_udp_port=14560+i, hil_mode=args.hitl, vehicle_type=vehicle_type,
        #                         model_source=sim_source, model_name=args.model_name, x=x, y=y, z=z,
        #                         roll=roll, pitch=pitch, yaw=yaw)
    elif sim == SimType.IGNITION:
        setup_ignition()
        # TODO: Figure out how to use multiple vehicles with HITL?
        lm.add_launch_description(
            spawn_ign_vehicle(namespace=namespace, instance=i, mavlink_tcp_port=4560+i,
                                mavlink_udp_port=14560+i, hil_mode=args.hitl, vehicle_type=vehicle_type,
                                api=api, model_source=sim_source, model_name=args.model_name, x=x, y=y, z=z,
                                roll=roll, pitch=pitch, yaw=yaw)
        )
    elif sim == SimType.AIRSIM:
        lm.print_info(f"AirSim settings/vehicle creation should be done from outside launch, assuming it is...")
    # Create executable call
    if api == ApiType.INHERENT:
        api_exe = sim.name.lower()
    else:
        api_exe = api.name.lower()
    vehicle_exe = f"{api_exe}_{vehicle_exe}"

    if api == ApiType.MAVROS:
        build_path=f"{os.environ['PX4_AUTOPILOT']}/build/px4_sitl_default"
        lm.add_action(
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
        lm.add_action(
            Node(
                package="mavros", executable="mavros_node",
                output="screen",
                namespace=f"{namespace}/mavros",
                # FIXME: parameters sometimes cause crash
                parameters=[
                    {
                        "fcu_url": f"udp://:{14540+i}@127.0.0.1:{14557+i}",
                        "gcs_url": "",
                        "target_system_id": i + 1,
                        "target_component_id": 1,
                        "fcu_protocol": "v2.0",
                    },
                    # os.path.join(ROBOT_CONTROL_PKG, "config", "px4_sim_params.yaml"),
                    # os.path.join(robot_control, "config", "px4_config.yaml"),
                    # os.path.join(robot_control, "config", "px4_pluginlists.yaml")
                ]
            )
        )
        # raise NotImplementedError("MAVROS api not implemented yet")
    elif api == ApiType.INHERENT:
        pass
    elif api == ApiType.NONE:
        return lm.describe_sub_entities()
    else:
        raise Exception(f"API {api.name} not supported yet")
    # Launch vehicle executable that creates common ROS2 API
    node_name = combine_names([namespace, "vehicle"], ".")
    lm.add_action(
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
                args.vehicle_yaml,
            ]
        ),
    )
    return lm.describe_sub_entities()


# Sources and their default models
GZ_MODEL_LIST = {
    SimSource.ROBOT.name: {
        VehicleType.DRONE.name: 'iris_realsense',
    },
    SimSource.NUAV.name: {
        VehicleType.DRONE.name: 'frog_v2',
        VehicleType.ROVER.name: 'bullfrog',
        VehicleType.PLANE.name: 'wallace',
    },
    SimSource.DEFAULT.name: {
        VehicleType.DRONE.name: 'iris',
        VehicleType.ROVER.name: 'r1_rover',
    }
}
def spawn_gz_vehicle(namespace="drone_0", instance=0, mavlink_tcp_port=4560, mavlink_udp_port=14560,
                     hil_mode=False, serial_device="/dev/ttyACM0", vehicle_type:VehicleType=VehicleType.DRONE,
                     model_source:SimSource=SimSource.DEFAULT, model_name="", x=0.0, y=0.0, z=0.0,
                     roll=0.0, pitch=0.0, yaw=0.0):
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
    lm = LaunchManager()
    # TODO: URDF still needs to be implemented
    # Creates tmp URDF file with frog v2
    mappings = {"namespace": namespace,
                "mavlink_tcp_port": mavlink_tcp_port,
                "mavlink_udp_port": mavlink_udp_port,
                "serial_enabled": "1" if hil_mode else "0",
                "serial_device": serial_device,
                "serial_baudrate": "921600",
                "hil_mode": "1" if hil_mode else "0"}
    mappings["camera_tilt"] = 0.0
    mappings["include_camera"] = True
    if model_name == "":
        if model_source.name not in GZ_MODEL_LIST:
            raise Exception(f"Model source {model_source.name} not supported")
        if vehicle_type.name not in GZ_MODEL_LIST[model_source.name]:
            raise Exception(f"Vehicle type {vehicle_type.name} not supported for source {model_source.name}")
        model_name = GZ_MODEL_LIST[model_source.name][vehicle_type.name]
    file_path = get_model_path("GAZEBO_MODEL_PATH", model_name)
    if file_path == "":
        raise Exception(f"Model {model_name} could not be found for gazebo")

    lm.print_info(f"Using model: {file_path}")
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
    lm.add_action(
        Node(
            package="gazebo_ros", executable="spawn_entity.py",
            arguments=[
                "-entity", namespace, "-file", tmp_path,
                "-robot_namespace", namespace,
                "-spawn_service_timeout", "120.0",
                "-x", str(x), "-y", str(y), "-z", str(z),
                "-R", str(roll), "-P", str(pitch), "-Y", str(yaw)
            ],
            output="screen"
        )
    )
    return lm


def get_main_msg(args):
    lm = LaunchManager()
    api = ApiType[args.api.upper()]
    main_msg = f'Spawning {args.vehicle_type} \'{args.namespace}\' - '
    main_parts = []
    main_parts.append(f'api: {args.api}')
    main_parts.append(f'sim: {args.sim}')
    main_parts.append(f'log_level: {args.log_level}')
    if args.model_name != "":
        main_parts.append(f'model: {args.model_name}')
    else:
        main_parts.append(f'sim_source: {args.model_name}')
    if args.hitl: main_parts.append(f'hitl')
    if args.camera: main_parts.append(f'camera')
    main_parts.append(f'api: {args.api}')
    if api == ApiType.MAVROS: main_parts.append(f'instance: {args.instance}')
    main_msg += ', '.join(main_parts)
    lm.print_info(main_msg)
    return lm


# Sources and their default models
IGN_MODEL_LIST = {
    SimSource.ROBOT.name: {
        VehicleType.DRONE.name: {
            ApiType.MAVROS.name: 'x3_mavlink',
            ApiType.INHERENT.name: 'x3_ignition',
        },
        VehicleType.ROVER.name: {
            ApiType.INHERENT.name: 'simple_rover_ignition',
        }
    },
}
def spawn_ign_vehicle(namespace="drone_0", instance=0, mavlink_tcp_port=4560, mavlink_udp_port=14560,
                      hil_mode=False, serial_device="/dev/ttyACM0", vehicle_type=VehicleType.DRONE,
                      api=ApiType.MAVROS, model_source=SimSource.DEFAULT, model_name="", x=0.0, y=0.0, z=0.0,
                     roll=0.0, pitch=0.0, yaw=0.0):
    """Spawns vehicle in running gazebo world.

    Args:
        namespace (str, optional): ROS namespace for all nodes. Defaults to "drone_0".
        instance (int, optional): Instance of PX4 SITL to start. Defaults to 0.
        mavlink_tcp_port (int, optional): TCP port to use with mavlink. Defaults to 4560.
        mavlink_udp_port (int, optional): UDP port to use with mavlink. Defaults to 14560.
        hil_mode (bool, optional): Flag that turns on HITL mode. Defaults to False.
        serial_device (str, optional): Path to PX4 serial device port. Defaults to "/dev/ttyACM0".
    """
    lm = LaunchManager()
    # TODO: URDF still needs to be implemented
    # Creates tmp URDF file with frog v2
    mappings = {"namespace": namespace,
                "mavlink_tcp_port": mavlink_tcp_port,
                "mavlink_udp_port": mavlink_udp_port,
                "serial_enabled": "1" if hil_mode else "0",
                "serial_device": serial_device,
                "serial_baudrate": "921600",
                "hil_mode": "1" if hil_mode else "0"}
    if model_name == "":
        if model_source.name not in IGN_MODEL_LIST:
            raise Exception(f"Model source {model_source.name} not supported")
        if vehicle_type.name not in IGN_MODEL_LIST[model_source.name]:
            raise Exception(f"Vehicle type {vehicle_type.name} not supported for source {model_source.name}")
        if api.name not in IGN_MODEL_LIST[model_source.name][vehicle_type.name]:
            raise Exception(f"API {api.name} not supported for vehicle type {vehicle_type.name} and source {model_source.name}")
        model_name = IGN_MODEL_LIST[model_source.name][vehicle_type.name][api.name]
    file_path = get_model_path("GZ_SIM_RESOURCE_PATH", model_name)
    if file_path == "":
        raise Exception(f"Model {model_name} could not be found for ignition")
    # Exposes ignition topics to ros
    ns = ""
    if namespace:
        ns = f"/{namespace}"
    bridge_args = []
    bridge_remaps = []
    # bridge_args += [
    #     # Realsense Camera Info
    #     f"/world/runway/model/{namespace}/link/realsense_d435/sensor/realsense_d435/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
    #     # Realsense Depth Camera  TODO: don't hardcode world runway name in ign topic
    #     f"/world/runway/model/{namespace}/link/realsense_d435/sensor/realsense_d435/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
    #     # Realsense Color Camera
    #     f"/world/runway/model/{namespace}/link/realsense_d435/sensor/realsense_d435/image@sensor_msgs/msg/Image[ignition.msgs.Image",
    #     # Realsense Point Cloud
    #     f"/world/runway/model/{namespace}/link/realsense_d435/sensor/realsense_d435/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
    # ]
    # bridge_remaps += [
    #     # Realsense Depth Camera
    #     (f"/world/runway/model/{namespace}/link/realsense_d435/sensor/realsense_d435/depth_image", f"/{namespace}/realsense/aligned_depth_to_color/image_raw"),
    #     # (f"", f"/{namespace}/realsense/aligned_depth_to_color/image_raw/compressed"),
    #     (f"/world/runway/model/{namespace}/link/realsense_d435/sensor/realsense_d435/camera_info", f"/{namespace}/realsense/aligned_depth_to_color/camera_info"),
    #     # Realsense Color Camera
    #     (f"/world/runway/model/{namespace}/link/realsense_d435/sensor/realsense_d435/image", f"/{namespace}/realsense/color/image_raw"),
    #     # (f"", f"/{namespace}/realsense/color/image_raw/compressed"),
    #     # (f"", f"/{namespace}/realsense/color/camera_info")
    #     # Realsense Point Cloud
    #     (f"/world/runway/model/{namespace}/link/realsense_d435/sensor/realsense_d435/points", f"/{namespace}/realsense/depth/color/points")
    # ]
    if api == ApiType.INHERENT:
        bridge_args += [
            # World info
            f"/world/empty/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            # Multicopter control
            f"{ns}/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist",
            f"{ns}/enable@std_msgs/msg/Bool]gz.msgs.Boolean",
            # Joint state publisher
            f"/world/empty/model{ns}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            # Odometry publisher
            f"/model{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        ]
        bridge_remaps += [
            # World info
            # (f"/world/empty/pose/info"),
            # Multicopter control
            (f"{ns}/command/twist", f"{ns}/_sim/command/twist"),
            (f"{ns}/enable", f"{ns}/_sim/enable"),
            # Joint state publisher
            (f"/world/empty/model{ns}/joint_state", f"{ns}/_sim/joint_state"),
            # Odometry publisher
            (f"/model{ns}/odom", f"{ns}/_sim/odom"),
        ]
    lm.add_action(
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            arguments=bridge_args,
            remappings=bridge_remaps,
            output="screen"
        ),
        # Node(package="ros2_utils", executable="relay", namespace=namespace,
        #     arguments=[f"realsense/aligned_depth_to_color/camera_info", f"realsense/color/camera_info"]
        # )
    )

    lm.print_info(f"Using model: {file_path}")
    tmp_path, robot_desc = parse_model_file(file_path, mappings)

    # Spawns vehicle model using SDF or URDF file
    lm.add_action(
        Node(
            package="ros_gz_sim", executable="create",
            arguments=[
                "-file", tmp_path,
                "-name", namespace,    
                "-x", str(x), "-y", str(y), "-z", str(z),
                "-R", str(roll), "-P", str(pitch), "-Y", str(yaw)
            ],
            output="screen"
        )
    )
    return lm


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


def generate_launch_description():
    """Launch this launch file."""
    lm = LaunchManager()
    lm.add_arg("namespace", "", "Vehicle namespace. Will autofill if not given. Ex: drone_0.")
    lm.add_arg("log_level", "debug", "Sets log level of ROS nodes.")
    lm.add_arg("sim", "ignition", "Simulation being used.", [e.name.lower() for e in SimType])
    lm.add_arg("vehicle_type", "drone", "Type of vehicle to spawn.", [e.name.lower() for e in VehicleType])
    lm.add_arg("model_name", "", "Model to spawn in ignition or gazebo. Overrides sim_source.")
    lm.add_arg("vehicle_yaml", DEFAULT_VEHICLE_YAML, "YAML file to load vehicle parameters from.")
    lm.add_arg("camera", "False", "Flag indicating whether to use a camera.")
    lm.add_arg("api", "inherent", "API to use.", [e.name.lower() for e in ApiType])
    lm.add_arg("x", "nan", "X position (m)")
    lm.add_arg("y", "nan", "Y position (m)")
    lm.add_arg("z", "nan", "Z position (m)")
    lm.add_arg("roll", "0.0", "Roll angle (deg)")
    lm.add_arg("pitch", "0.0", "Pitch angle (deg)")
    lm.add_arg("yaw", "0.0", "Yaw angle (deg)")
    # MAVROS
    lm.add_arg("hitl", "False", "Flag to enable HITL")
    lm.add_arg("instance", "1", "Vehicle instance. Used for calculating mavros variables.")
    # CONFIG
    lm.add_arg("sim_source", "robot", "Source for simulation.", [e.name.lower() for e in SimSource])
    lm.add_opaque_function(launch_setup)
    return lm
