#!/usr/bin/env python3
'''
common.py
=======================

Common functions for completing `VehicleClient` futures.
'''
from typing import List, Type
from rclpy.duration import Duration
from rclpy.action.client import GoalStatus
from robot_control.cli.vehicle_client import VehicleClient
from rclpy.executors import Executor
from rclpy.task import Future
import functools


def complete_action_call(node: VehicleClient, executor: Type[Executor], future, action_name: str):
    def get_result_cb(future):
        node.get_logger().info(f"Got result for '{action_name}'")
    executor.spin_until_future_complete(future)
    goal_handle = None
    start_time = node.get_clock().now()
    while goal_handle is None:
        if node.get_clock().now() - start_time > Duration(seconds=2.0):
            msg = f"Timed out waiting for goal handle: '{action_name}'"
            node.get_logger().error(msg)
            return False
        executor.spin_once()
        goal_handle = node._goal_handles.get(action_name)
    gh_future = goal_handle.get_result_async()
    gh_future.add_done_callback(get_result_cb)
    executor.spin_until_future_complete(gh_future)
    if goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
        node.get_logger().info(f"Succeeded at '{action_name}'!")
        return True
    return False


#TODO: reimplement below, figure out if a new node is needed for logging (don't use 'drone_0' hardcode)
def complete_clients(executor: Executor, clients, futures: List[Future], action_names: List[str]):
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
    for future in futures: executor.spin_once_until_future_complete(future)


def check_futures_done(futures: List[Future]):
    for future in futures:
        if not future.done():
            return False
    return True


def check_goal_handles_success(goal_handles: List):
    for goal_handle in goal_handles:
        if goal_handle.status != GoalStatus.STATUS_SUCCEEDED:
            return False
    return True


def check_for_nones(ls: List):
    for i in ls:
        if i is None:
            return True
    return False