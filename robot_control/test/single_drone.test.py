#!/usr/bin/env python3
'''
Run `ros2 test single_drone.test.py`
'''
from launch import LaunchDescription
from launch_ros.actions import Node as lNode
import launch_testing

import rclpy
from rclpy.action.client import ClientGoalHandle
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.task import Future
from robot_control.launch.run_vehicles import generate_airsim

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from robot_control_interfaces.action import ArmTakeoff, Land

import pytest
import unittest
import functools


NAMESPACE = "drone_0"
@pytest.mark.launch_test
def generate_test_description():
    ld = []
    airsim_drone = lNode(
        package='robot_control',
        executable='airsim_drone',
        namespace=NAMESPACE,
        output='screen',
        arguments=["--ros-args", "--log-level", f"{NAMESPACE}.vehicle:=DEBUG"]
    )

    context = {'airsim_drone': airsim_drone}

    ld += [airsim_drone]
    ld += generate_airsim(namespaces=[NAMESPACE], environment="D:\\git\\external\\AirSim\\Unreal\\Environments\\Blocks426\\output\\Blocks\\run.bat")
    # Start tests right away - no need to wait for anything
    ld += [launch_testing.actions.ReadyToTest()]

    return LaunchDescription(ld), context


# @launch_testing.post_shutdown_test()
# class TestProcessOutput(unittest.TestCase):
 
#     def test_exit_code(self, proc_output, proc_info, airsim_drone):
#         # Check that process exits with code -15 code: termination request, sent to the program
#         launch_testing.asserts.assertExitCodes(proc_info, [-15], process=airsim_drone)


class DroneClient(Node):
    def __init__(self, test_cls: unittest.TestCase):
        super().__init__("drone_client", namespace=NAMESPACE)
        # self._default_callback_group = ReentrantCallbackGroup()
        self._sub_pose = self.create_subscription(PoseStamped, "pose", self._cb_pose, 10)
        self._cli_arm_takeoff = ActionClient(self, ArmTakeoff, "arm_takeoff")
        self._cli_land = ActionClient(self, Land, "land")
        self._pose = PoseStamped()
        self._poses_received = 0
        self.test_cls = test_cls

        # Goal handles
        self._goal_handles = {}

    def _cb_pose(self, msg: PoseStamped):
        self._pose = msg
        self._poses_received += 1

    def send_arm_takeoff(self, altitude: float):
        if not self._cli_arm_takeoff.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("No action server available")
            return
        goal = ArmTakeoff.Goal(altitude=altitude)
        self.get_logger().info(f"Sending goal to arm and takeoff")
        future = self._cli_arm_takeoff.send_goal_async(goal)
        future.add_done_callback(functools.partial(self._action_response, "arm_takeoff"))
        return future

    def _action_response(self, action_name: str, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected for '{action_name}'")
            return
        self.get_logger().info(f"Goal accepted for '{action_name}'")
        self._goal_handles[action_name] = goal_handle

    def send_land(self):
        if not self._cli_land.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("No action server available")
            return
        goal = Land.Goal()
        self.get_logger().info(f"Sending goal to land")
        future = self._cli_land.send_goal_async(goal)
        future.add_done_callback(functools.partial(self._action_response, "land"))
        return future


class TestArmTakeoffLand(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # cls.context = Context()
        rclpy.init()
        cls.nh = DroneClient(cls)
 
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.addCleanup(self.nh.destroy_subscription, self.nh._sub_pose)
        self.addCleanup(self.nh._cli_arm_takeoff.destroy)

    def complete_action_call(self, executor, future, action_name, required=True):
        def get_result_cb(future):
            self.nh.get_logger().info(f"Got result for '{action_name}'")
        executor.spin_until_future_complete(future)
        goal_handle = None
        start_time = self.nh.get_clock().now()
        while goal_handle is None:
            if self.nh.get_clock().now() - start_time > Duration(seconds=2.0):
                msg = f"Timed out waiting for goal handle: '{action_name}'"
                if required:
                    self.fail(msg)
                else:
                    self.nh.get_logger().error(msg)
                    return False
            executor.spin_once()
            goal_handle = self.nh._goal_handles.get(action_name)
        gh_future = goal_handle.get_result_async()
        gh_future.add_done_callback(get_result_cb)
        executor.spin_until_future_complete(gh_future)
        if goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
            self.nh.get_logger().info(f"Succeeded at '{action_name}'!")
            return True
        return False

    # https://answers.ros.org/question/322831/ros2-wait-for-action-using-rclpy/
    def test_arm_takeoff_land(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self.nh)

        while self.nh._poses_received <= 10:
            self.nh.get_logger().info("Spinning til 10 poses are received.")
            executor.spin_once()

        # TODO: check that initial position is same after takeoff and land
        try:
            # Takeoff drone
            future = self.nh.send_arm_takeoff(altitude=3.0)
            self.assertTrue(self.complete_action_call(executor, future, "arm_takeoff"))

            # Land drone 
            future = self.nh.send_land()
            executor.spin_until_future_complete(future)
            self.assertTrue(self.complete_action_call(executor, future, "land"))
        finally:
            executor.remove_node(self.nh)


if __name__ == '__main__':
    unittest.main()
