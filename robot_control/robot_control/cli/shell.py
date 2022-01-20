#!/usr/bin/env python3
'''
Shell
======
Shell interface for calling ROS2 actions.
'''
import rclpy
from cmd2 import Cmd2ArgumentParser, with_argparser
from .common import complete_action_call, ClientShell
from .drone_client import DroneClient
from ..utils.structs import Frame
from argparse import ArgumentParser
from rclpy.parameter import Parameter
from rcl_interfaces.msg import Parameter as ParameterMsg
from pprint import pprint
import numpy as np
import argparse

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def xyz_yaw_type(s):
    try:
        x, y, z, yaw = map(float, s.split(','))
        return [x, y, z, np.deg2rad(yaw)]
    except:
        raise argparse.ArgumentTypeError("Must be x,y,z,yaw")


# https://pymotw.com/2/cmd/
# https://pypi.org/project/cmd2/
class DroneShell(ClientShell):
    prompt = "> "
    intro = "Welcome to drone shell! Type ? to list commands"
    def __init__(self, name) -> None:
        super().__init__(name, DroneClient, persistent_history_file='~/.robot_control/cmd2_history.dat')
        self._update_params_choices()

    _go_coord_argparser = Cmd2ArgumentParser(description='Sends `go_waypoint` action.')
    _go_coord_argparser.add_argument('x', type=float, help='x (m)')
    _go_coord_argparser.add_argument('y', type=float, help='y (m)')
    _go_coord_argparser.add_argument('z', type=float, help='z (m)')
    _go_coord_argparser.add_argument('yaw', type=float, help='yaw (deg)')
    _go_coord_argparser.add_argument('frame', type=int, choices=[e.value for e in Frame], help=f'frame mapping ' + str({e.name: e.value for e in Frame}))
    @with_argparser(_go_coord_argparser)
    def do_go_wp(self, opts):
        future = self.client.send_go_waypoint(opts.x, opts.y, opts.z, np.deg2rad(opts.yaw), opts.frame)
        complete_action_call(self.client, self.executor, future, "go_waypoint")

    # Safety wrapper to always send FRD on field, since any other could cause unknown results
    _go_frd_argparser = Cmd2ArgumentParser(description='Sends `go_waypoint` action with FRD.')
    _go_frd_argparser.add_argument('x', type=float, help='x (m)')
    _go_frd_argparser.add_argument('y', type=float, help='y (m)')
    _go_frd_argparser.add_argument('z', type=float, help='z (m)')
    _go_frd_argparser.add_argument('yaw', type=float, help='yaw (deg)')
    @with_argparser(_go_frd_argparser)
    def do_go_frd(self, opts):
        future = self.client.send_go_waypoint(opts.x, opts.y, opts.z, np.deg2rad(opts.yaw), Frame.FRD)
        complete_action_call(self.client, self.executor, future, "go_waypoint")

    _takeoff_argparser = Cmd2ArgumentParser(description='Sends `arm_takeoff` action.')
    _takeoff_argparser.add_argument('alt', type=float, help='height (m)')
    @with_argparser(_takeoff_argparser)
    def do_takeoff(self, opts):
        future = self.client.send_arm_takeoff(opts.alt)
        complete_action_call(self.client, self.executor, future, "arm_takeoff")

    def do_land(self, opts):
        """Sends `land` action."""
        future = self.client.send_land()
        complete_action_call(self.client, self.executor, future, "land")

    def do_arm(self, opts):
        """Sends `arm` service."""
        resp = self.client.send_arm()

    def do_disarm(self, opts):
        """Sends `disarm` service."""
        resp = self.client.send_disarm()

    def do_kill(self, opts):
        """Sends `kill` service."""
        resp = self.client.send_kill()
        
    _run_tree_argparser = Cmd2ArgumentParser(description="Sends `run_tree` action.")
    _run_tree_argparser.add_argument("-t", "--tree", default="", type=str, help='BT file to read from. Uses default in action when empty.')
    @with_argparser(_run_tree_argparser)
    def do_run_tree(self, opts):
        future = self.client.send_run_tree(opts.tree)
        complete_action_call(self.client, self.executor, future, "run_tree")

    _pub_frd_argparser = Cmd2ArgumentParser(description='Publishes `cmd/frd` pose.')
    _pub_frd_argparser.add_argument('x', type=float, help='x (m)')
    _pub_frd_argparser.add_argument('y', type=float, help='y (m)')
    _pub_frd_argparser.add_argument('z', type=float, help='z (m)')
    _pub_frd_argparser.add_argument('yaw', type=float, help='yaw (deg)')
    @with_argparser(_pub_frd_argparser)
    def do_pub_frd(self, opts):
        self.client.publish_pose(opts.x,opts.y,opts.z,np.deg2rad(opts.yaw),Frame.FRD)
    
    _pub_ned_argparser = Cmd2ArgumentParser(description='Publishes `cmd/ned` pose.')
    _pub_ned_argparser.add_argument('x', type=float, help='x (m)')
    _pub_ned_argparser.add_argument('y', type=float, help='y (m)')
    _pub_ned_argparser.add_argument('z', type=float, help='z (m)')
    _pub_ned_argparser.add_argument('yaw', type=float, help='yaw (deg)')
    @with_argparser(_pub_ned_argparser)
    def do_pub_ned(self, opts):
        self.client.publish_pose(opts.x,opts.y,opts.z,np.deg2rad(opts.yaw),Frame.LOCAL_NED)

    _pub_path_ned_argparser = Cmd2ArgumentParser(description='Publishes `cmd/path/ned` path.')
    _pub_path_ned_argparser.add_argument('xyz_yaw', type=xyz_yaw_type, help='x(m),y(m),z(m),yaw(deg)', nargs="+")
    @with_argparser(_pub_path_ned_argparser)
    def do_pub_path_ned(self, opts):
        path = np.asfarray(opts.xyz_yaw)
        self.client.publish_path(path, Frame.LOCAL_NED)

    _pub_vel_argparser = Cmd2ArgumentParser(description='Publishes `cmd/frd` pose.')
    _pub_vel_argparser.add_argument('vx', type=float, help='x (m/s)')
    _pub_vel_argparser.add_argument('vy', type=float, help='y (m/s)')
    _pub_vel_argparser.add_argument('vz', type=float, help='z (m/s)')
    _pub_vel_argparser.add_argument('yaw_rate', type=float, help='yaw (rad/s)')
    @with_argparser(_pub_vel_argparser)
    def do_pub_vel(self, opts):
        self.client.publish_vel(opts.vx,opts.vy,opts.vz,opts.yaw_rate)

    _set_parameters_argparser = Cmd2ArgumentParser(description="Sends `set_parameters` service.")
    _set_parameters_name_arg = _set_parameters_argparser.add_argument("name", type=str, help="Name of parameter.")
    _set_parameters_argparser.add_argument("value", help="Value of parameter.")
    @with_argparser(_set_parameters_argparser)
    def do_set_param(self, opts):
        resp = self.client.send_set_parameter(opts.name, opts.value)

    _get_parameters_argparser = Cmd2ArgumentParser(description="Sends `get_parameters` service.")
    _get_parameters_name_arg = _get_parameters_argparser.add_argument("name", type=str, help="Name of parameter.")
    @with_argparser(_get_parameters_argparser)
    def do_get_param(self, opts):
        resp = self.client.send_get_parameter(opts.name)
        for param in resp.values:
            pmsg = ParameterMsg(value=param)
            p = Parameter.from_parameter_msg(pmsg)
            print(f"{opts.name}: {p.value}")

    def do_get_all_params(self, opts):
        """Gets list of all parameters."""
        names = self.client.send_list_parameters().result.names
        resp = self.client.send_get_parameters(names)
        d = {}
        for param, name in zip(resp.values, names):
            pmsg = ParameterMsg(value=param, name=name)
            p = Parameter.from_parameter_msg(pmsg)
            d[name] = p.value
        pprint(d)

    def do_list_params(self, opts):
        """Gets list of all parameters."""
        names = self.client.send_list_parameters().result.names
        pprint(names)

    _set_params_node_name_argparser = Cmd2ArgumentParser(description="Changes node to read parameter from.")
    _set_params_node_name_name_arg = _set_params_node_name_argparser.add_argument("name", type=str, help="Name of parameter node.")
    @with_argparser(_set_params_node_name_argparser)
    def do_set_params_node_name(self, opts):
        self.client.params_node_name = opts.name
        self._update_params_choices()

    def _update_params_choices(self):
        names = list(self.client._node_parameters.keys())
        node_names_tuples = self.client.get_node_names_and_namespaces()
        filtered_tuples = []
        for pair in node_names_tuples:
            if pair[1] == f"/{self.client.namespace}":
                filtered_tuples.append(pair)
        node_names_d = dict(filtered_tuples)
        self._node_names = node_names_d
        self._set_params_node_name_name_arg.choices = list(node_names_d.keys())
        self._get_parameters_name_arg.choices = names
        self._set_parameters_name_arg.choices = names


def main(args=None):
    rclpy.init(args=args)

    parser = ArgumentParser()
    parser.add_argument("-n", "--name", default='drone_0', help='initial vehicle name')
    args, _ = parser.parse_known_args()

    shell = DroneShell(args.name)
    shell.cmdloop()


if __name__=="__main__":
    main()
