#!/usr/bin/env python3
import numpy as np
from ros2_utils import angular_dist


class XYZYaw:
    def __init__(self, x, y, z, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

    def __str__(self):
        return f"x: {self.x}, y: {self.y}, z: {self.z}, yaw: {self.yaw}"

    @property
    def xyz(self):
        return np.asfarray([self.x, self.y, self.z])

    @xyz.setter
    def xyz(self, value):
        if len(value) != 3:
            raise ValueError("must be length 3")
        self.x = value[0]
        self.y = value[1]
        self.z = value[2]

    @property
    def xyz_yaw(self):
        return np.asfarray([self.x, self.y, self.z, self.yaw])

    @xyz_yaw.setter
    def xyz_yaw(self, value):
        if len(value) != 4:
            raise ValueError("must be length 4")
        self.x = value[0]
        self.y = value[1]
        self.z = value[2]
        self.yaw = value[3]

    @property
    def xy(self):
        return np.asfarray([self.x, self.y])


class Constraints:
    def __init__(self, max_vel_horz_abs, max_vel_vert_abs, max_yaw_rate):
        self.max_vel_horz_abs = max_vel_horz_abs
        self.max_vel_vert_abs = max_vel_vert_abs
        self.max_yaw_rate = max_yaw_rate


# Derived from AirSim PIDPositionController
# https://github.com/microsoft/AirSim/blob/master/ros/src/airsim_ros_pkgs/src/pd_position_controller_simple.cpp
class PIDPositionController:
    def __init__(self, kp, ki, kd, max_vel_horz_abs, max_vel_vert_abs, max_yaw_rate):
        """Generates velocity commands to move towards a position.

        Args:
            kp (np.ndarray): Array containing x, y, z, and yaw p values.
            kd (np.ndarray): Array containing x, y, z, and yaw d values.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = XYZYaw(0.0, 0.0, 0.0, 0.0)
        self.prev_error = XYZYaw(0.0, 0.0, 0.0, 0.0)
        self.curr_error = XYZYaw(0.0, 0.0, 0.0, 0.0)
        self.curr_position = XYZYaw(np.nan, np.nan, np.nan, np.nan)
        self.target_position = XYZYaw(np.nan, np.nan, np.nan, np.nan)

        self.vel_cmd = XYZYaw(np.nan, np.nan, np.nan, np.nan)
        self.constraints = Constraints(max_vel_horz_abs, max_vel_vert_abs, max_yaw_rate)

    def update(self) -> XYZYaw:
        self.compute_control_cmd()
        self.enforce_dynamic_constraints()
        return self.vel_cmd

    def compute_control_cmd(self):
        self.curr_error.xyz = self.target_position.xyz - self.curr_position.xyz
        self.curr_error.yaw = angular_dist(self.curr_position.yaw, self.target_position.yaw)

        self.integral.xyz_yaw += self.ki * self.curr_error.xyz_yaw
        vel_norm_horz = np.linalg.norm(self.integral.xy)
        if vel_norm_horz > self.constraints.max_vel_horz_abs:
            self.integral.x = (self.integral.x / vel_norm_horz) * self.constraints.max_vel_horz_abs
            self.integral.y = (self.integral.y / vel_norm_horz) * self.constraints.max_vel_horz_abs
        if np.abs(self.integral.z) > self.constraints.max_vel_vert_abs:
            self.integral.z = (self.integral.z / np.abs(self.integral.z)) * self.constraints.max_vel_vert_abs
        if np.abs(self.integral.yaw) > self.constraints.max_yaw_rate:
            self.integral.yaw = (self.integral.yaw / np.abs(self.integral.yaw)) * self.constraints.max_yaw_rate
        
        p_term = self.kp * self.curr_error.xyz_yaw
        d_term = self.kd * self.prev_error.xyz_yaw

        self.prev_error = self.curr_error

        self.vel_cmd.xyz_yaw = p_term + d_term + self.integral.xyz_yaw

    def enforce_dynamic_constraints(self):
        vel_norm_horz = np.linalg.norm(self.vel_cmd.xy)
        if vel_norm_horz > self.constraints.max_vel_horz_abs:
            self.vel_cmd.x = (self.vel_cmd.x / vel_norm_horz) * self.constraints.max_vel_horz_abs
            self.vel_cmd.y = (self.vel_cmd.y / vel_norm_horz) * self.constraints.max_vel_horz_abs
        
        if np.abs(self.vel_cmd.z) > self.constraints.max_vel_vert_abs:
            self.vel_cmd.z = (self.vel_cmd.z / np.abs(self.vel_cmd.z)) * self.constraints.max_vel_vert_abs

        if np.abs(self.vel_cmd.yaw) > self.constraints.max_yaw_rate:
            self.vel_cmd.yaw = (self.vel_cmd.yaw / np.abs(self.vel_cmd.yaw)) * self.constraints.max_yaw_rate

    def reset_errors(self):
        self.prev_error = XYZYaw(0.0, 0.0, 0.0, 0.0)

    def set_current(self, x, y, z, yaw):
        self.curr_position.xyz_yaw = [x, y, z, yaw]

    def set_target(self, x, y, z, yaw):
        self.target_position.xyz_yaw = [x, y, z, yaw]
