#!/usr/bin/env python3
import numpy as np
from enum import IntEnum
from geometry_msgs.msg import Point, Vector3, Quaternion, Pose, Twist
from nav_msgs.msg import Odometry
from robot_control_interfaces.msg import Waypoint
from scipy.spatial.transform import Rotation as R


class Frame(IntEnum):
    LLA = Waypoint.LLA
    LOCAL_NED = Waypoint.LOCAL_NED
    FRD = Waypoint.FRD


# Useful for identifying reference axis or planes
class Axis(IntEnum):
    X = 0
    Y = 1
    Z = 2
    XY = 3
    XZ = 4
    YZ = 5
    XYZ = 6


# Convert enum to mask
AXIS_TO_MASK = {
    Axis.X: [1, 0, 0],
    Axis.Y: [0, 1, 0],
    Axis.Z: [0, 0, 1],
    Axis.XY: [1, 1, 0],
    Axis.XZ: [1, 0, 1],
    Axis.YZ: [0, 1, 1],
    Axis.XYZ: [1, 1, 1],
}


class NpVector3(np.ndarray):
    def __new__(cls, vector3):
        cls.__check_v3_size(cls, vector3)
        obj = np.asfarray(vector3).view(cls)
        return obj

    @classmethod
    def xyz(cls, x, y, z) -> 'NpVector3':
        return cls.__new__(cls, [x, y, z])

    @property
    def x(self):
        return self[0]

    @x.setter
    def x(self, value):
        self[0] = value

    @property
    def y(self):
        return self[1]

    @y.setter
    def y(self, value):
        self[1] = value

    @property
    def z(self):
        return self[2]

    @z.setter
    def z(self, value):
        self[2] = value

    @property
    def v3(self):
        return self[:3]
    
    @v3.setter
    def v3(self, value):
        if isinstance(value, (Vector3, Point)):
            self.x = value.x
            self.y = value.y
            self.z = value.z
        else:
            self.__check_v3_size(value)
            self[:3] = np.asfarray(value)[:]
    
    def get_point_msg(self):
        msg = Point()
        msg.x = self[0]
        msg.y = self[1]
        msg.z = self[2]
        return msg

    def get_vector3_msg(self):
        msg = Vector3()
        msg.x = self[0]
        msg.y = self[1]
        msg.z = self[2]
        return msg

    def __check_v3_size(self, vector3):
        if len(vector3) != 3:
            raise ValueError("must contain 3 values in array.")


class NpVector4(np.ndarray):
    def __new__(cls, quat):
        cls.__check_v4_size(cls, quat)
        obj = np.asfarray(quat).view(cls)
        return obj

    @classmethod
    def xyzw(cls, x, y, z, w) -> 'NpVector4':
        return cls.__new__(cls, [x, y, z, w])

    @property
    def x(self):
        return self[0]

    @x.setter
    def x(self, value):
        self[0] = value

    @property
    def y(self):
        return self[1]

    @y.setter
    def y(self, value):
        self[1] = value

    @property
    def z(self):
        return self[2]

    @z.setter
    def z(self, value):
        self[2] = value

    @property
    def w(self):
        return self[3]

    @w.setter
    def w(self, value):
        self[3] = value

    @property
    def v4(self):
        return self[:4]

    @v4.setter
    def v4(self, value):
        if isinstance(value, Quaternion):
            self.x = value.x
            self.y = value.y
            self.z = value.z
            self.w = value.w
        else:
            self.__check_v4_size(value)
            self[:4] = np.asfarray(value)[:]

    @property
    def euler(self):
        return NpVector3(R.from_quat(self.v4).as_euler('xyz'))

    @euler.setter
    def euler(self, value):
        self.v4 = R.from_euler('xyz', value).as_quat()
    
    def get_quat_msg(self):
        msg = Quaternion()
        msg.x = self[0]
        msg.y = self[1]
        msg.z = self[2]
        msg.w = self[3]
        return msg

    def __check_v4_size(self, quat):
        if len(quat) != 4:
            raise ValueError("must contain 4 values in array.")


class NpPose:
    def __init__(self, position: NpVector3, orientation: NpVector4):
        self._position = position
        self._orientation = orientation

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        self._position.v3 = value

    @property
    def orientation(self):
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        self._orientation.v4 = value

    @property
    def euler(self):
        return self.orientation.euler

    @euler.setter
    def euler(self, value):
        self._orientation.euler = value
    
    def set_pose(self, value: Pose):
        self.position = value.position
        self.orientation = value.orientation

    def get_msg(self) -> Pose:
        msg = Pose()
        msg.position = self.position.get_point_msg()
        msg.orientation = self.orientation.get_quat_msg()
        return msg


class NpTwist:
    def __init__(self, linear: NpVector3, angular: NpVector3):
        self._linear = linear
        self._angular = angular

    @property
    def linear(self):
        return self._linear
    
    @linear.setter
    def linear(self, value):
        self._linear.v3 = value

    @property
    def angular(self):
        return self._angular
    
    @angular.setter
    def angular(self, value):
        self._angular.v3 = value

    @property
    def vx(self):
        return self._linear.x

    @vx.setter
    def vx(self, value):
        self._linear.x = value

    @property
    def vy(self):
        return self._linear.y

    @vy.setter
    def vy(self, value):
        self._linear.y = value

    @property
    def vz(self):
        return self._linear.z

    @vz.setter
    def vz(self, value):
        self._linear.z = value

    def set_msg(self, value: Twist):
        self.linear = value.linear
        self.angular = value.angular

    def get_msg(self) -> Twist:
        msg = Twist()
        msg.linear = self._linear.get_vector3_msg()
        msg.angular = self._angular.get_vector3_msg()

    
class NpOdometry:
    def __init__(self, pose: NpPose, twist: NpTwist):
        self.pose = pose
        self.twist = twist

    @property
    def vx(self):
        return self.twist.vx

    @vx.setter
    def vx(self, value):
        self.twist.vx = value

    @property
    def vy(self):
        return self.twist.vy

    @vy.setter
    def vy(self, value):
        self.twist.vy = value

    @property
    def vz(self):
        return self.twist.vz

    @vz.setter
    def vz(self, value):
        self.twist.vz = value

    def set_msg(self, value: Odometry):
        self.pose.set_pose(value.pose.pose)
        self.twist.set_twist(value.twist.twist)

    def get_msg(self) -> Odometry:
        msg = Odometry()
        msg.pose.pose = self.pose.get_msg()
        msg.twist.twist = self.twist.get_msg()
