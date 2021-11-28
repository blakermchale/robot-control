#!/usr/bin/env python3
from enum import IntEnum
from robot_control_interfaces.msg import Waypoint


class Frame(IntEnum):
    LLA = Waypoint.LLA
    LOCAL_NED = Waypoint.LOCAL_NED
    FRD = Waypoint.FRD
