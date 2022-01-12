#!/usr/bin/env python3
from enum import IntEnum, auto
from ament_index_python.packages import get_package_share_directory


# Get relative package directories
ROBOT_CONTROL_PKG = get_package_share_directory("robot_control")

# API's and the simulators they work with
API_PAIRS = {
    "mavros": ["airsim", "gazebo", "ignition", "none"],
    "inherent": ["airsim", "ignition"],
    "none": ["airsim", "gazebo", "ignition", "none"],
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
    INHERENT=auto()
    MAVROS=auto()


class SimSource(IntEnum):
    DEFAULT=0
    ROBOT=auto()
    NUAV=auto()


class ScenarioType(IntEnum):
    NONE=0
    DARKNET=1
