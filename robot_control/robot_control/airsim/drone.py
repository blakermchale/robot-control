#!/usr/bin/env python
# This package
from robot_control.abstract_drone import ADrone
from robot_control.airsim.vehicle import Vehicle
from robot_control.airsim.my_multirotor_client import MyMultirotorClient
# ROS libraries
import rclpy
from rclpy.executors import MultiThreadedExecutor
# Common packages
import numpy as np


class Drone(ADrone, Vehicle):
    def __init__(self, log_level="info", instance=0):
        super().__init__(log_level=log_level, instance=instance)
        self._client = MyMultirotorClient(self.namespace)
        print("Drone")

    def update(self):
        self._client.update_state()
        self._client.update_rotor_states()
        self._position = self._client.get_position()
        self._orientation = self._client.get_orientation()
        super().update()

    #####################
    ## Control commands
    #####################
    def arm(self):
        return self._client.arm()
    
    def disarm(self):
        return self._client.disarm()

    def halt(self):
        x = self._position[0]
        y = self._position[1]
        z = self._position[2]
        self._client.move_position(x, y, z, 0).join()  # TODO: use vehicle heading

    def land(self):
        self._client.land()

    def takeoff(self, alt: float):
        goal_alt = self._position[2] + -1*alt
        self.set_target(self._position[0], self._position[1], goal_alt)
        self._client.takeoff(goal_alt)

    def send_waypoint(self, x: float, y: float, z: float, heading: float, frame: int):
        # TODO: handle frame conversions
        self.set_target(x, y, z)
        self._client.move_position(x, y, z, heading)

    #######################
    ## Checking states
    #######################
    def is_landed(self):
        return self._client.is_landed()

    def is_armed(self):
        speeds = []
        for rotor in self._client._rotor_states.rotors:
            speeds.append(rotor["speed"])
        armed = not np.isclose(speeds, 0).all()
        self.get_logger().debug(f"Rotor speeds: {speeds}, Armed: {armed}", throttle_duration_sec=2.0)
        return armed


def main(args=None):
    rclpy.init(args=args)
    drone = Drone()
    executor = MultiThreadedExecutor()
    rclpy.spin(drone, executor=executor)


if __name__=="__main__":
    main()
