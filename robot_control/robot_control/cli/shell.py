#!/usr/bin/env python3
import rclpy
from cmd2 import Cmd, Cmd2ArgumentParser, with_argparser
from rclpy.executors import MultiThreadedExecutor
from robot_control.cli.common import complete_action_call
from robot_control.cli.drone_client import DroneClient


# https://pymotw.com/2/cmd/
# https://pypi.org/project/cmd2/
class DroneShell(Cmd):
    prompt = "> "
    intro = "Welcome! Type ? to list commands"
    def __init__(self) -> None:
        super().__init__()
        self.client = DroneClient(namespace="drone_0")
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.client)

    go_wp_argparser = Cmd2ArgumentParser(description='Sends `go_to_coordinate` action.')
    go_wp_argparser.add_argument('x', type=float, help='x (m)')
    go_wp_argparser.add_argument('y', type=float, help='y (m)')
    go_wp_argparser.add_argument('z', type=float, help='z (m)')
    go_wp_argparser.add_argument('yaw', type=float, help='yaw (rad)')
    go_wp_argparser.add_argument('frame', type=int, help='frame')
    @with_argparser(go_wp_argparser)
    def do_go_wp(self, opts):
        future = self.client.send_go_waypoint(opts.x, opts.y, opts.z, opts.yaw, opts.frame)
        complete_action_call(self.client, self.executor, future, "go_waypoint")

    takeoff_argparser = Cmd2ArgumentParser(description='Sends `arm_takeoff` action.')
    takeoff_argparser.add_argument('alt', type=float, help='height (m)')
    @with_argparser(takeoff_argparser)
    def do_takeoff(self, opts):
        future = self.client.send_arm_takeoff(opts.alt)
        complete_action_call(self.client, self.executor, future, "arm_takeoff")

    def do_land(self, opts):
        """Sends `land` action."""
        future = self.client.send_land()
        complete_action_call(self.client, self.executor, future, "land")
        
    def do_exit(self, args):
        """Exit shell."""
        print("Exiting")
        return True

    def default(self, inp):
        if inp in ["x", "q"]:
            return self.do_exit(inp)
        print("Default not implemented: {}".format(inp))

    do_EOF = do_exit


def main(args=None):
    rclpy.init(args=args)
    shell = DroneShell()
    shell.cmdloop()


if __name__=="__main__":
    main()
