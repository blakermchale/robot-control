#!/usr/bin/env python
# This package
from robot_control import Frame, Axes
from robot_control.abstract_drone import ADrone
from robot_control.airsim.vehicle import Vehicle
from robot_control.airsim.my_multirotor_client import MyMultirotorClient
# ROS libraries
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.clock import Duration
from airsim_ros.convert import ros_from_multirotor_state
# ROS interfaces
from airsim_ros_interfaces.msg import MultirotorState
# Common packages
import numpy as np
from argparse import ArgumentParser


# FIXME: a new client has to be instantiated for each call to drone to prevent exception https://github.com/microsoft/AirSim/issues/2607
class Drone(ADrone, Vehicle):
    def __init__(self, instance=0):
        super().__init__(instance=instance)
        self.get_logger().info("Creating AirSim client")
        self._client = MyMultirotorClient(self._namespace)
        self._pub_airsim_multirotor_state = self.create_publisher(MultirotorState, "airsim/multirotor_state", 10)
        # FIXME: temporary fix using join method to find when land finishes
        self._landed = True
        self.get_logger().info("AirSim Drone initialized")

    def update(self):
        self._client.update_state()
        self._client.update_rotor_states()
        self.position = self._client.get_position()
        self.orientation = self._client.get_orientation()
        self._publish_airsim()
        super().update()

    #####################
    ## Control commands
    #####################
    def arm(self):
        client = MyMultirotorClient(self._namespace)
        return client.arm()
        # return self._client.arm()
    
    def disarm(self):
        client = MyMultirotorClient(self._namespace)
        result = client.disarm()
        # FIXME: not necessary once landed state is fixed
        if result: self._landed = True
        return result
        # return self._client.disarm()

    def halt(self):
        x = self.position.x
        y = self.position.y
        z = self.position.z
        client = MyMultirotorClient(self._namespace)
        client.move_position(x, y, z, 0).join()
        # self._client.move_position(x, y, z, 0).join()  # TODO: use vehicle heading

    def land(self):
        client = MyMultirotorClient(self._namespace)
        init_pos = self.position.copy()
        land_count = 0
        max_lands = 5
        while not self.has_moved(init_pos, Axes.Z):
            init_pos = self.position.copy()
            self.get_logger().debug(f"calling async #{land_count}")
            future = client.land()
            future.join()
            if land_count >= max_lands:
                return False
            land_count += 1
        self._landed = True
        return True
        # self._client.land()

    def takeoff(self, alt: float):
        goal_alt = self.position.z + -1*alt
        self.set_target(self.position.x, self.position.y, goal_alt, self.euler.z)
        client = MyMultirotorClient(self._namespace)
        client.takeoff(goal_alt)
        return True
        # self._client.takeoff(goal_alt)

    def send_waypoint(self, x: float, y: float, z: float, heading: float, frame: Frame = Frame.LOCAL_NED):
        try:
            x, y, z, roll, pitch, heading = self.convert_position_frame(x, y, z, 0, 0, heading, frame, Frame.LOCAL_NED)
        except Exception as e:
            self.get_logger().error(str(e))
            return False
        self.set_target(x, y, z, heading)
        client = MyMultirotorClient(self._namespace)
        client.move_position(x, y, z, heading)
        # self._client.move_position(x, y, z, heading)
        return True

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float, frame: Frame = Frame.FRD):
        # TODO: test using same client as update loop here and make sure no exceptions occur when it is called quickly
        if frame == Frame.LOCAL_NED:
            self._client.move_local_velocity(vx, vy, vz, yaw_rate)
        elif frame == Frame.FRD:
            self._client.move_body_velocity(vx, vy, vz, yaw_rate)
        else:
            self.get_logger().error(f"Frame {frame.name} is not supported")
            return False
        return True

    #######################
    ## Actions
    #######################
    def _success_arm_takeoff(self):
        self._landed = False

    #######################
    ## Checking states
    #######################
    def is_landed(self):
        landed = self._client.is_landed()
        landed = self._landed
        # collision = self._client._state.collision.has_collided
        # self.get_logger().debug(f"Landed: {landed}, Collision: {collision}")
        return landed

    def is_armed(self):
        speeds = []
        for rotor in self._client._rotor_states.rotors:
            speeds.append(rotor["speed"])
        armed = not np.isclose(speeds, 0).all()
        # self.get_logger().debug(f"Rotor speeds: {speeds}, Armed: {armed}", throttle_duration_sec=2.0)
        return armed

    #######################
    ## Publishers
    #######################
    def _publish_airsim(self):
        msg = ros_from_multirotor_state(self._client._state)
        self._pub_airsim_multirotor_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    # Setup argument parsing
    parser = ArgumentParser()
    parser.add_argument("-i", "--instance", default=0, type=int, help="Instance of vehicle.")
    args, _ = parser.parse_known_args()
    # Spin drone
    drone = Drone(instance=args.instance)
    executor = MultiThreadedExecutor()
    rclpy.spin(drone, executor=executor)


if __name__=="__main__":
    main()
