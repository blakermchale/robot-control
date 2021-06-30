#!/usr/bin/env python
from airsim import MultirotorClient, YawMode, DrivetrainType, LandedState, MultirotorState
import numpy as np
import os

from airsim.types import RotorStates
from msgpackrpc.future import Future

class MyMultirotorClient(MultirotorClient):
    """
    Client class for interfacing with airsim multirotor api.
    """
    def __init__(self, namespace):
        if not os.environ["WSL_HOST_IP"]:
            raise ValueError("Set WSL_HOST_IP env variable")
        ip = os.environ["WSL_HOST_IP"]
        super().__init__(ip=ip)
        self.confirmConnection()
        self._vehicle_name = namespace
        self.enableApiControl(True, vehicle_name=self._vehicle_name)
        self._state = MultirotorState()
        self._rotor_states = RotorStates()

    ########################
    ## Commands
    ########################
    def land(self) -> Future:
        """Asynchronously tells vehicle to land."""
        return self.landAsync(vehicle_name=self._vehicle_name)

    def takeoff(self, altitude: float) -> Future:
        """Asynchronously tells vehicle to takeoff."""
        # return self.takeoffAsync(vehicle_name=self._vehicle_name)  # it seems like drone actually only goes 1.5 m up instead of 3.0 m
        return self.moveToZAsync(altitude, 3.0, vehicle_name=self._vehicle_name)

    def arm(self):
        """Arms vehicle.
        
        Returns:
            bool: if arm was a success
        """
        return self.armDisarm(True, vehicle_name=self._vehicle_name)

    def disarm(self):
        """Disarms vehicle.
        
        Returns:
            bool: if disarm was a success
        """
        return self.armDisarm(False, vehicle_name=self._vehicle_name)

    def move_position(self, x: float, y: float, z: float, heading: float) -> Future:
        """Moves to position and heading.

        Args:
            x (float): x position meters
            y (float): y position meters
            z (float): z position meters
            heading (float): angle radians
        """
        yaw_mode = YawMode(is_rate=False, yaw_or_rate=np.rad2deg(heading))
        return self.moveToPositionAsync(x, y, z, 3.0, yaw_mode=yaw_mode, vehicle_name=self._vehicle_name)

    def move_local_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float) -> Future:
        """Moves by velocity in world NED frame.

        Args:
            vx (float): x velocity m/s
            vy (float): y velocity m/s
            vz (float): z velocity m/s
            yaw_rate (float): yaw rate rad/s
        """
        # TODO: figure out how long duration should be
        yaw_mode = YawMode(is_rate=True, yaw_or_rate=np.rad2deg(yaw_rate))
        return self.moveByVelocityAsync(vx, vy, vz, 0.1, yaw_mode=yaw_mode, vehicle_name=self._vehicle_name)
    
    def move_body_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float) -> Future:
        """Moves by velocity in body NED frame.

        Args:
            vx (float): x velocity m/s
            vy (float): y velocity m/s
            vz (float): z velocity m/s
            yaw_rate (float): yaw rate rad/s
        """
        # TODO: figure out how long duration should be
        yaw_mode = YawMode(is_rate=True, yaw_or_rate=np.rad2deg(yaw_rate))
        return self.moveByVelocityBodyFrameAsync(vx, vy, vz, 0.1, yaw_mode=yaw_mode, vehicle_name=self._vehicle_name)

    ########################
    ## Checking states
    ########################
    def is_landed(self):
        """If vehicle has landed.
        
        Returns:
            bool: Whether the vehicle has landed
        """
        # FIXME: Landed state is not set automatically because of bug https://github.com/microsoft/AirSim/issues/1776
        state = self._state.landed_state
        return state == LandedState.Landed
        # return self._state.collision.has_collided

    ########################
    ## Get states
    ########################
    def update_state(self):
        """Updates multirotor state. Meant to be called in one thread only.

        Returns:
            MultirotorState: state of multirotor
        """
        self._state = self.getMultirotorState(vehicle_name=self._vehicle_name)
        return self._state

    def update_rotor_states(self):
        """Updates rotor states. Meant to be called in one thread only.

        Returns:
            RotorStates: state of rotors
        """
        self._rotor_states = self.getRotorStates(vehicle_name=self._vehicle_name)
        return self._rotor_states

    def get_position(self):
        """Gets current position of drone.
        
        Returns:
            np.ndarray: position x,y,z in meters
        """
        kin_pos = self._state.kinematics_estimated.position
        position = np.asfarray([kin_pos.x_val, kin_pos.y_val, kin_pos.z_val])
        return position

    def get_orientation(self):
        """Gets current orientation of drone.
        
        Returns:
            np.ndarray: quaternion x,y,z,w
        """
        kin_orientation = self._state.kinematics_estimated.orientation
        orientation = np.asfarray([kin_orientation.x_val, kin_orientation.y_val, kin_orientation.z_val, kin_orientation.w_val])
        return orientation
