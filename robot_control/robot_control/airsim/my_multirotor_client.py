#!/usr/bin/env python
from airsim import MultirotorClient, YawMode, DrivetrainType, LandedState, MultirotorState
import numpy as np
import os

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

    def land(self):
        """Asynchronously tells vehicle to land."""
        return self.landAsync(vehicle_name=self._vehicle_name)

    def is_landed(self):
        """If vehicle has landed.
        
        Returns:
            bool: Whether the vehicle has landed
        """
        state = self._state.landed_state
        return state == LandedState.Landed

    def takeoff(self):
        """Asynchronously tells vehicle to takeoff."""
        return self.takeoffAsync(vehicle_name=self._vehicle_name)

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

    def update_state(self):
        """Updates multirotor state. Meant to be called in one thread only.

        Returns:
            MultirotorState: state of multirotor
        """
        self._state = self.getMultirotorState(vehicle_name=self._vehicle_name)
        return self._state

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
