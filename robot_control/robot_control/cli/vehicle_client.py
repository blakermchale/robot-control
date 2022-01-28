#!/usr/bin/env python3
'''
Vehicle Client
=======================

Generic vehicle client to ROS2 actions and topics. Easy publishing, calling,
and sending goals.
'''
import numpy as np
import time
import enum
from typing import Any, List
from rclpy.executors import Executor
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Twist, Pose, Point
from robot_control_interfaces.msg import Waypoint
from robot_control_interfaces.action import FollowWaypoints, GoWaypoint, RunBT
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters, DescribeParameters
from rcl_interfaces.msg import Parameter as ParameterMsg
from nav_msgs.msg import Path
from std_msgs.msg import Header

from .common import get_parameter_value_msg_from_type, setup_send_action, NodeClient
from ..utils.structs import Frame
from ros2_utils import NpVector4


class VehicleClient(NodeClient):
    def __init__(self, executor: Executor, namespace=None, log_feedback=True):
        super().__init__("vehicle_client", executor, namespace=namespace)

        # Vehicle state
        self._sub_pose = self.create_subscription(PoseStamped, "pose", self._cb_pose, 10)
        # Vehicle control
        self._cli_go_waypoint = ActionClient(self, GoWaypoint, "go_waypoint")
        self._cli_follow_waypoints = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self._cli_arm = self.create_client(Trigger, "arm")
        self._cli_disarm = self.create_client(Trigger, "disarm")
        self._cli_kill = self.create_client(Trigger, "kill")
        self._cli_run_tree = ActionClient(self, RunBT, "run_tree")
        self._pub_cmd_vel = self.create_publisher(Twist, "cmd/velocity", 1)
        self._pub_cmd_ned = self.create_publisher(Pose, "cmd/ned", 1)
        self._pub_cmd_frd = self.create_publisher(Pose, "cmd/frd", 1)
        self._pub_cmd_path_ned = self.create_publisher(Path, "cmd/path/ned", 1)
        # ROS parameter setting and getting
        self._params_node_name = "vehicle"
        self._set_params_srvs()

        # Internal states
        self._pose = PoseStamped()
        self._poses_received = 0
        self.log_feedback = log_feedback

        # Pre-load parameters assuming types will not change
        self._node_parameters = {}
        self.update_node_parameters()

    def update_node_parameters(self):
        list_params = self.send_list_parameters()
        desc_params = self.send_describe_parameters(list_params.result.names)
        # FIXME: create issue or post for `describe_parameters` service being broken, currently returns
        #  duplicates names when a list of unique names is given. seems to replace other parameters with
        #  copies of others
        self._node_parameters = {}
        for i, name in enumerate(list_params.result.names):
            self._node_parameters[name] = desc_params.descriptors[i]

    def send_go_waypoint(self, x: float, y: float, z: float, heading: float, frame):
        @setup_send_action(self, self._cli_go_waypoint, self._feedback_go_waypoint)
        def send_action():
            goal = GoWaypoint.Goal()
            goal.waypoint.frame = frame
            goal.waypoint.heading = heading
            goal.waypoint.position.x = x
            goal.waypoint.position.y = y
            goal.waypoint.position.z = z
            return goal
        return send_action

    def send_run_tree(self, behavior_tree: str):
        @setup_send_action(self, self._cli_run_tree, self._feedback_run_tree)
        def send_action():
            goal = RunBT.Goal()
            if behavior_tree != "":
                goal.behavior_tree = behavior_tree
            return goal
        return send_action

    def send_follow_waypoints(self, xyz_yaw_frame: np.ndarray, tolerance: float):
        """Sends follow_waypoints action to vehicle namespace.

        Args:
            xyz_yaw_frame (np.ndarray): np array with first 4 values as x,y,z,yaw and las
            tolerance (float): Tolerance to accept a waypoint.

        Returns:
            Future: future object for action
        """
        @setup_send_action(self, self._cli_follow_waypoints, self._feedback_follow_waypoints)
        def send_action():
            waypoints = []
            for row in xyz_yaw_frame:
                waypoints.append(Waypoint(position=Point(x=row[0],y=row[1],z=row[2]),heading=row[3],frame=int(row[4])))
            goal = FollowWaypoints.Goal()
            goal.waypoints = waypoints
            goal.tolerance = tolerance
            return goal
        return send_action

    def send_arm(self):
        if not self._cli_arm.wait_for_service(timeout_sec=self._timeout_sec):
            self.get_logger().error("No service available")
            return
        req = Trigger.Request()
        future = self._cli_arm.call_async(req)
        self._executor.spin_until_future_complete(future)
        return future.result()

    def send_disarm(self):
        if not self._cli_disarm.wait_for_service(timeout_sec=self._timeout_sec):
            self.get_logger().error("No service available")
            return
        req = Trigger.Request()
        future = self._cli_disarm.call_async(req)
        self._executor.spin_until_future_complete(future)
        return future.result()

    def send_kill(self):
        if not self._cli_kill.wait_for_service(timeout_sec=self._timeout_sec):
            self.get_logger().error("No service available")
            return
        req = Trigger.Request()
        future = self._cli_kill.call_async(req)
        self._executor.spin_until_future_complete(future)
        return future.result()

    def send_set_parameter(self, name: str, value: Any):
        if not self._cli_set_param.wait_for_service(timeout_sec=self._timeout_sec):
            self.get_logger().error("No service available")
            return
        param_type = self._node_parameters[name].type
        param_value = get_parameter_value_msg_from_type(param_type, value)
        req = SetParameters.Request()
        req.parameters = [ParameterMsg(name=name, value=param_value)]
        future = self._cli_set_param.call_async(req)
        self._executor.spin_until_future_complete(future)
        return future.result()

    def send_get_parameter(self, name: str):
        if not self._cli_get_param.wait_for_service(timeout_sec=self._timeout_sec):
            self.get_logger().error("No service available")
            return
        req = GetParameters.Request()
        req.names = [name]
        future = self._cli_get_param.call_async(req)
        self._executor.spin_until_future_complete(future)
        return future.result()

    def send_get_parameters(self, names: List[str]):
        if not self._cli_get_param.wait_for_service(timeout_sec=self._timeout_sec):
            self.get_logger().error(f"No service available {self._cli_get_param.srv_name}")
            return
        req = GetParameters.Request()
        req.names = names
        future = self._cli_get_param.call_async(req)
        self._executor.spin_until_future_complete(future)
        return future.result()

    def send_list_parameters(self):
        if not self._cli_list_param.wait_for_service(timeout_sec=self._timeout_sec):
            self.get_logger().error(f"No service available {self._cli_list_param.srv_name}")
            return
        future = self._cli_list_param.call_async(ListParameters.Request())
        self._executor.spin_until_future_complete(future)
        return future.result()

    def send_describe_parameters(self, names):
        if not self._cli_desc_param.wait_for_service(timeout_sec=self._timeout_sec):
            self.get_logger().error(f"No service available {self._cli_desc_param.srv_name}")
            return
        future = self._cli_desc_param.call_async(DescribeParameters.Request(names=names))
        self._executor.spin_until_future_complete(future)
        return future.result()

    def publish_vel(self, vx, vy, vz, yaw_rate):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        msg.angular.z = yaw_rate
        
        try:
            while True:
                self._pub_cmd_vel.publish(msg)
                time.sleep(0.1)  # TODO: don't use time, use rate from rclpy
        except KeyboardInterrupt:
            pass

    def publish_pose(self, x: float, y: float, z: float, heading: float, frame: Frame):
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.orientation = NpVector4.from_rpy(0.,0.,heading).get_quat_msg()
        if frame == Frame.FRD:
            pub = self._pub_cmd_frd
        elif frame == Frame.LOCAL_NED:
            pub = self._pub_cmd_ned
        else:
            self.get_logger().error(f"Invalid frame for shell: {frame.name}")
            return
        try:
            while True:
                pub.publish(msg)
                time.sleep(0.1)  # TODO: don't use time, use rate from rclpy
        except KeyboardInterrupt:
            pass

    def publish_path(self, xyz_yaw: np.ndarray, frame: Frame):
        msg = Path()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        if frame == Frame.LOCAL_NED:
            for e in xyz_yaw:
                p = PoseStamped()
                p.pose.position.x = e[0]
                p.pose.position.y = e[1]
                p.pose.position.z = e[2]
                p.pose.orientation = NpVector4.from_rpy(0.,0.,e[3]).get_quat_msg()
                p.header = header
                msg.poses.append(p)
            msg.header = header
            self._pub_cmd_path_ned.publish(msg)
        else:
            raise ValueError(f"Frame {frame.name} not supported for path publisher yet")

    @property
    def params_node_name(self):
        return self._params_node_name

    @params_node_name.setter
    def params_node_name(self, value):
        self._params_node_name = value
        self._set_params_srvs()
        self.update_node_parameters()
        
    def _set_params_srvs(self):
        self._cli_set_param = self.create_client(SetParameters, f"{self.params_node_name}/set_parameters")
        self._cli_get_param = self.create_client(GetParameters, f"{self.params_node_name}/get_parameters")
        self._cli_list_param = self.create_client(ListParameters, f"{self.params_node_name}/list_parameters")
        self._cli_desc_param = self.create_client(DescribeParameters, f"{self.params_node_name}/describe_parameters")

    ########################
    ## Subscribers
    ########################
    def _cb_pose(self, msg: PoseStamped):
        self._pose = msg
        self._poses_received += 1

    #######################
    ## Feedback callbacks
    #######################
    def _feedback_follow_waypoints(self, feedback):
        self.feedback = feedback.feedback
        if self.log_feedback: self.get_logger().info(f"`follow_waypoints` feedback: {feedback.feedback.distance}m, idx: {feedback.feedback.idx}", throttle_duration_sec=2.0)

    def _feedback_go_waypoint(self, feedback):
        self.feedback = feedback.feedback
        if self.log_feedback: self.get_logger().info(f"`go_waypoint` feedback: {feedback.feedback.distance}m", throttle_duration_sec=2.0)

    def _feedback_run_tree(self, feedback):
        self.feedback = feedback.feedback
        if self.log_feedback: self.get_logger().info(f"`run_tree` feedback: running for {feedback.feedback.time.sec}s", throttle_duration_sec=2.0)
