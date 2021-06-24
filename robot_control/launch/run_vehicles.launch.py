#!/usr/bin/env python3
'''
start_vehicles.launch.py

Starts specified number of vehicles in simulation with ardupilot connection and
basic_drone control system. If is_sim is set to false it will not start the
simulation.
'''


from launch import launch_description
import launch_ros
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir

from ament_index_python.packages import get_package_share_directory
import os
import jinja2
import xacro
from enum import IntEnum

from airsim_utils.generate_settings import create_settings
from airsim_utils.generate_settings import VehicleType as AirSimVehicleType
from airsim_utils.run_environment import run_environment

# Get relative package directories
robot_control = get_package_share_directory("robot_control")


class VehicleType(IntEnum):
    DRONE = 0
    ROVER = 1
    PLANE = 2


class SimType(IntEnum):
    NONE = 0
    GAZEBO = 1
    AIRSIM = 2


def generate_launch_description():   
    launch_description = []
    launch_description += get_launch_arguments()
    launch_description += [OpaqueFunction(function = launch_setup)] 
    return LaunchDescription(launch_description)


def get_launch_arguments():
    return [
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Starts gazebo gui',
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Starts gazebo server with verbose outputs',
        ),
        DeclareLaunchArgument(
            'world',
            default_value="franklin_park.world",
            description='Starts gazebo server with world file',
        ),
        DeclareLaunchArgument(
            'nb',
            default_value="1",
            description='Number of vehicles to spawn.',
        ),
        DeclareLaunchArgument(
            'base_name',
            default_value="",
            description='Prefix for all vehicles.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='debug',
            description='Sets log level of ros nodes.',
        ),
        DeclareLaunchArgument(
            'sim',
            default_value="AIRSIM",
            choices=[e.name for e in SimType],
            description="Whether or not to start gazebo with this script"
        ),
        DeclareLaunchArgument(
            'vehicle_type',
            default_value="drone",
            description="Type of vehicle to spawn."
        ),
        DeclareLaunchArgument("fsw", default_value="PX4", description="Whether to use PX4")
    ]


def launch_setup(context, *args, **kwargs):
    """Allows declaration of launch arguments within the ROS2 context
    """
    nb = int(LaunchConfiguration('nb').perform(context))
    log_level = LaunchConfiguration('log_level').perform(context)
    base_name = LaunchConfiguration('base_name').perform(context)
    vehicle_type = LaunchConfiguration('vehicle_type').perform(context)
    sim = LaunchConfiguration('sim').perform(context)
    hitl = False  # TODO: implement hitl arg

    # Check for empty variables
    if not os.environ["PX4_AUTOPILOT"]:
        raise Exception("PX4_AUTOPILOT env variable must be set")

    # Choose base name according to vehicle type
    if base_name == "":
        base_name = vehicle_type
    vehicle_type = VehicleType[vehicle_type.upper()]
    sim = SimType[sim.upper()]

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

    ld = []
    # Start simulator
    if sim == SimType.GAZEBO:
        ld.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [robot_control, os.path.sep, 'launch',
                        os.path.sep, 'gazebo.launch.py']
                ),
                launch_arguments={
                    'verbose': LaunchConfiguration('verbose'),
                    'gui': LaunchConfiguration('gui'),
                }.items(),
            )
        )
        vehicle_exe = f"mavros_{vehicle_exe}"
    elif sim == SimType.AIRSIM:
        environment = r"D:\git\external\AirSim\Unreal\Environments\Blocks426\output\Blocks\run.bat"  # TODO: don't hardcode
        # Generate settings
        if hitl:
            # FIXME: issue with using custom LLA during HITL, just use default
            create_settings(nb=nb, pawn_bp="Class'/Game/NUAV/Blueprints/BP_FlyingPawn.BP_FlyingPawn_C'",
                            hitl=hitl)
        else:
            create_settings(nb=nb, pawn_bp="Class'/Game/NUAV/Blueprints/BP_FlyingPawn.BP_FlyingPawn_C'",
                            lat=42.3097, lon=-71.0959, alt=141.0, hitl=hitl, vehicle_type=AirSimVehicleType.SimpleFlight)
            os.environ["PX4_SIM_HOST_ADDR"] = os.environ["WSL_HOST_IP"]
        run_env = run_environment(env=environment)
        if not run_env:
            return ld
        vehicle_exe = f"airsim_{vehicle_exe}"

    # instance = 0
    # build_path=f"{os.environ['PX4_AUTOPILOT']}/build/px4_sitl"
    # log_directory = f"{build_path}/instance_0"
    # namespace = "drone_0"
    # ld.append(
    #     launch_ros.actions.Node(
    #         package='px4_computing', executable="sitl",
    #         output='screen',
    #         namespace=namespace,
    #         arguments=[
    #             "--log-level", log_level,
    #             "--instance", str(instance)
    #         ],
    #     ),
    # )

    namespace = "drone_0"  # TODO: don't hardcode
    instance = 0  # TODO: don't hardcode
    # spawn_vehicle(ld, build_path=build_path, log_directory=log_directory)
    # TODO: re-enable
    # Launch basic_drone file that starts all topics, services, and actions to control drone through ROS
    # ld.append(
    #     launch_ros.actions.Node(
    #         package='robot_control', executable=vehicle_exe,
    #         output='screen',
    #         namespace=namespace,
    #         arguments=[
    #             "--log-level", log_level,
    #             "--instance", str(instance)
    #         ],
    #     ),
    # )

    # Spawn Vehicles
    # for i in range(nb):
    #     log_directory = f"{build_path}/instance_{i}"
    #     namespace = f"{base_name}_{i}"
    #     # TODO: Figure out how to use multiple vehicles with HITL?
    #     spawn_vehicle(ld, namespace=namespace, instance=i, log_directory=log_directory,
    #                   mavlink_tcp_port=4560+i, mavlink_udp_port=14560+i, log_level=log_level,
    #                   hil_mode=hil_mode, serial_device=serial_device, gazebo=gazebo, 
    #                   darknet=darknet,vehicle_type=vehicle_type)

    return ld


def spawn_vehicle(launch_description, namespace="drone_0", instance=0, log_directory=f"{os.environ['HOME']}/logs",
                  mavlink_tcp_port=4560, mavlink_udp_port=14560, log_level=None,
                  hil_mode=False, serial_device="/dev/ttyACM0", vehicle_type=VehicleType.DRONE):
    """Spawns vehicle in running gazebo world with PX4 SITL, micrortps agent and client, and basic_vehicle.

    Args:
        launch_description (list): Launch description list to append nodes and other launch files to.
        namespace (str, optional): ROS namespace for all nodes. Defaults to "drone_0".
        instance (int, optional): Instance of PX4 SITL to start. Defaults to 0.
        mavlink_tcp_port (int, optional): TCP port to use with mavlink. Defaults to 4560.
        mavlink_udp_port (int, optional): UDP port to use with mavlink. Defaults to 14560.
        log_level (str, optional): Log level to start nodes at in ROS. Nodes must have an implemented way to take this value. Defaults to None.
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
        vehicle_exe = 'drone'
    elif vehicle_type == VehicleType.ROVER:
        file_path = f'{os.environ["PX4_AUTOPILOT"]}/Tools/sitl_gazebo/models/r1_rover/r1_rover.sdf.jinja'
        vehicle_exe = "rover"
    elif vehicle_type == VehicleType.PLANE:
        file_path = f'{os.environ["PX4_AUTOPILOT"]}/Tools/sitl_gazebo/models/plane/plane.sdf.jinja'
        vehicle_exe = "plane"
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

    # Spawns vehicle model using SDF or URDF file
    launch_description.append(
        launch_ros.actions.Node(
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
