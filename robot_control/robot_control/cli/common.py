#!/usr/bin/env python3
'''
Common
=======================

Common functions for completing `VehicleClient` futures.
'''
from typing import List, Type
from rclpy.duration import Duration
from rclpy.action.client import GoalStatus
from rclpy.executors import Executor
from rclpy.task import Future
from rclpy.node import Node
import functools
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterValue
from cmd2 import Cmd, Cmd2ArgumentParser, with_argparser
from rclpy.executors import MultiThreadedExecutor


class NodeClient(Node):
    def __init__(self, node_name: str, executor: Executor, namespace=None):
        super().__init__(node_name, namespace=namespace)
        self.namespace = self.get_namespace().split("/")[-1]

        # Internal states
        self._timeout_sec = 60.0
        self._waiting_for_gh = False

        # Goal handles
        self._goal_handles = {}

        self._executor = executor
        self._executor.add_node(self)
    
    ########################
    ## Helpers
    ########################
    def reset_actions(self):
        # self._goal_handles = {}
        self._waiting_for_gh = True

    def _action_response(self, action_name: str, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected for '{action_name}'")
            self._waiting_for_gh = False
            return
        self.get_logger().info(f"Goal accepted for '{action_name}'")
        self._goal_handles[action_name] = goal_handle


class ClientShell(Cmd):
    """Generic client shell that handles cancelling actions. Assumes field `self.client` is set to a child of `NodeClient`."""
    def __init__(self, name, ClientObj: NodeClient, **kwargs) -> None:
        # print(kwargs)
        super().__init__(**kwargs)
        self.ClientObj = ClientObj
        client = self.ClientObj(MultiThreadedExecutor(), namespace=name)
        self.clients_archive = {name: client}
        self.client = client
        self.name = name

    _set_name_argparser = Cmd2ArgumentParser(description='Changes client to new vehicle name.')
    _set_name_argparser.add_argument('name', type=str, help='vehicle namespace')
    @with_argparser(_set_name_argparser)
    def do_set_name(self, opts):
        if opts.name not in self.clients_archive.keys():
            self.clients_archive[opts.name] = self.ClientObj(MultiThreadedExecutor(), namespace=opts.name)
        self.client = self.clients_archive[opts.name]
        self.name = opts.name

    def do_get_name(self, opts):
        """Gets current client vehicle name."""
        print(f"Vehicle name is '{self.client.namespace}'")

    def sigint_handler(self, signum: int, _) -> None:
        cancel_futures = []
        if self.client._waiting_for_gh:
            print("Cannot cancel until goal is retrieved")
            return
        for k, v in list(self.client._goal_handles.items()):
            cancel_futures.append(v.cancel_goal_async())
            self.client._goal_handles.pop(k)
            print(f"\nCancelling `{k}`!")
        while self.executor._context.ok() and not check_futures_done(cancel_futures) and not self.executor._is_shutdown:
            self.executor.spin_once()
        if cancel_futures:
            print("Finished ^C")
        super().sigint_handler(signum, _)

    def do_exit(self, args):
        """Exit shell."""
        print("Exiting")
        return True

    def default(self, inp):
        if inp in ["x", "q"]:
            return self.do_exit(inp)
        print("Default not implemented: {}".format(inp))

    do_EOF = do_exit

    @property
    def executor(self):
        return self.client.executor


def setup_send_action(self, action_cli, feedback_cb):
    def inner(func):
        if action_cli._action_name in self._goal_handles:
            self.get_logger().error(f"`{action_cli._action_name}` is still being sent")
            return
        self.reset_actions()
        if not action_cli.wait_for_server(timeout_sec=self._timeout_sec):
            self.get_logger().error(f"No action server available for `{action_cli._action_name}`")
            return
        goal = func()
        self.get_logger().info(f"Sending goal to `{action_cli._action_name}`")
        future = action_cli.send_goal_async(goal, feedback_callback=feedback_cb)
        future.add_done_callback(functools.partial(self._action_response, action_cli._action_name))
        return future
    return inner


def complete_action_call(node, executor: Type[Executor], future, action_name: str):
    """Sends single action call."""
    if future is None:
        return False
    def get_result_cb(future):
        node.get_logger().info(f"Got result for '{action_name}'")
    executor.spin_until_future_complete(future)
    goal_handle = None
    while goal_handle is None:
        executor.spin_once()
        goal_handle = node._goal_handles.get(action_name)
    node._waiting_for_gh = False  # Makes sure goal handle is retrieved before cancelling
    gh_future = goal_handle.get_result_async()
    gh_future.add_done_callback(get_result_cb)
    executor.spin_until_future_complete(gh_future)
    node._goal_handles.pop(action_name)  # remove from active handles
    if goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
        node.get_logger().info(f"Succeeded at '{action_name}'!")
        return True
    return False


#TODO: reimplement below, figure out if a new node is needed for logging (don't use 'drone_0' hardcode)
def complete_clients(executor: Executor, clients, futures: List[Future], action_names: List[str]):
    """Waits til multiple action calls complete."""
    if len(futures) != len(action_names) or len(futures) != len(clients.keys()):
        raise Exception("Number of clients, futures, and action names must match")
    def get_result_cb(node, action_name, future):
        node.get_logger().info(f"Got result for '{action_name}'")
    # Spin until all futures complete
    while executor._context.ok() and not check_futures_done(futures) and not executor._is_shutdown:
        clients["drone_0"].get_logger().info("waiting for futures", throttle_duration_sec=2.0)
        executor.spin_once()
        # spin_until_futures_complete(executor, futures)
    # Wait for all goal handles to be set
    goal_handles = [None] * len(futures)
    start_time = clients["drone_0"].get_clock().now()
    while check_for_nones(goal_handles):
        if clients["drone_0"].get_clock().now() - start_time > Duration(seconds=2.0):
            msg = f"Timed out waiting for goal handles"
            clients["drone_0"].get_logger().error(msg)
            return False
        executor.spin_once()
        for i, client in enumerate(clients.values()):
            goal_handles[i] = client._goal_handles.get(action_names[i])
    
    # Establish goal handle futures with callbacks
    gh_futures = [goal_handle.get_result_async() for goal_handle in goal_handles]
    ls_clients = list(clients.values())
    for i, goal_handle in enumerate(goal_handles):
        gh_future = goal_handle.get_result_async()
        gh_future.add_done_callback(functools.partial(get_result_cb, ls_clients[i], action_names[i]))
        gh_futures.append(gh_future)
    # Wait for goal handle futures to finish
    while executor._context.ok() and not check_futures_done(gh_futures) and not executor._is_shutdown:
        clients["drone_0"].get_logger().info("waiting for goal handle futures", throttle_duration_sec=2.0)
        executor.spin_once()
    if check_goal_handles_success(goal_handles):
        clients["drone_0"].get_logger().info(f"Succeeded with actions!")
        return True
    return False


def spin_until_futures_complete(executor: Executor, futures: List[Future]):
    """Spins executor until list of futures complete."""
    for future in futures: executor.spin_once_until_future_complete(future)


def check_futures_done(futures: List[Future]):
    """Checks that list of futures are all done."""
    for future in futures:
        if not future.done():
            return False
    return True


def check_goal_handles_success(goal_handles: List):
    """Checks list of goal handles for success."""
    for goal_handle in goal_handles:
        if goal_handle.status != GoalStatus.STATUS_SUCCEEDED:
            return False
    return True


def check_for_nones(ls: List):
    """Checks for nones in list."""
    for i in ls:
        if i is None:
            return True
    return False


def get_parameter_value_msg_from_type(type_, value):
    """Converts a type enum and value to its corresponding `ParameterValue` msg."""
    param_msg = ParameterValue(type=type_)
    if Parameter.Type.BOOL.value == type_:
        param_msg.bool_value = bool(value)
    elif Parameter.Type.INTEGER.value == type_:
        param_msg.integer_value = int(value)
    elif Parameter.Type.DOUBLE.value == type_:
        param_msg.double_value = float(value)
    elif Parameter.Type.STRING.value == type_:
        param_msg.string_value = str(value)
    elif Parameter.Type.BYTE_ARRAY.value == type_:
        param_msg.byte_array_value = value
    elif Parameter.Type.BOOL_ARRAY.value == type_:
        param_msg.bool_array_value = value
    elif Parameter.Type.INTEGER_ARRAY.value == type_:
        param_msg.integer_array_value = value
    elif Parameter.Type.DOUBLE_ARRAY.value == type_:
        param_msg.double_array_value = value
    elif Parameter.Type.STRING_ARRAY.value == type_:
        param_msg.string_array_value = value
    return param_msg
