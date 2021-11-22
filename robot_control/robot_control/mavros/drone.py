#!/usr/bin/env python
# This package
from robot_control.abstract_drone import ADrone
from robot_control.mavros.vehicle import Vehicle
# ROS libraries
import rclpy
from rclpy.executors import MultiThreadedExecutor
from mavros_msgs.srv import CommandTOL
# Common packages
from argparse import ArgumentParser


class Drone(ADrone, Vehicle):
    def __init__(self, instance=0):
        super().__init__( instance=instance)
        
        # ROS2 services
        self._cli_takeoff = self.create_client(CommandTOL, "mavros/cmd/takeoff")
        self._cli_land = self.create_client(CommandTOL, "mavros/cmd/land")

        self.get_logger().debug("Initialized Drone!")

    ######################
    ## Control commands ##
    ######################
    def takeoff(self, alt: float):
        req = CommandTOL.Request()
        if self._amsl_alt is None:
            self.get_logger().error("Takeoff must have AMSL altitude set")
            return False
        req.altitude = float(alt) + self._amsl_alt # Set target altitude
        req.latitude = float("nan") # nan tells it to use current lat/lon/yaw
        req.longitude = float("nan")
        req.yaw = float("nan")
        self.set_target(self.position.x, self.position.y, alt, self.euler.z, to_ned=True)  # TODO: is this NED?
        resp = self._cli_takeoff.call(req)
        if not resp.success:
            self.get_logger().error(f"Failed to takeoff (MAV_RESULT={resp.result})")
            return False
        return True

    def land(self):
        req = CommandTOL.Request()
        req.latitude = float("nan")
        req.longitude = float("nan")
        req.yaw = float("nan")
        resp = self._cli_land.call(req)
        if not resp.success:
            self.get_logger().error(f"Failed to land (MAV_RESULT={resp.result})")
            return False
        return True

    #####################
    ## Checking states ##
    #####################
    def is_landed(self):
        return not self._state.armed


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
