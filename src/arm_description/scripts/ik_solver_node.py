#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""URDF 位置反解服务。

订阅 /robot_description，提供 /solve_ik 服务（moveit_msgs/GetPositionIK）。
请求里的末端位置用米，内部转换成 cm；返回的关节角用弧度。
只求解 gripper_frame_link 运动链上的 5 个臂关节，gripper 保持请求 seed 里的值。
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String

from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK

import arm_kinematics


class IkSolverNode(Node):
    def __init__(self):
        super().__init__("arm_ik_solver")

        self.declare_parameter("target_link", "gripper_frame_link")
        self.declare_parameter("tolerance_cm", 0.5)
        self.declare_parameter("max_iterations", 300)
        self.declare_parameter("restarts", 10)

        self.target_link = self.get_parameter("target_link").value
        self.tolerance_cm = float(self.get_parameter("tolerance_cm").value)
        self.max_iterations = int(self.get_parameter("max_iterations").value)
        self.restarts = int(self.get_parameter("restarts").value)

        self.model = None
        self.chain_names = []

        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String, "robot_description", self.on_robot_description, qos
        )
        self.create_service(GetPositionIK, "/solve_ik", self.handle_ik)

        self.get_logger().info(
            f"等待 /robot_description；目标 link = {self.target_link}"
        )

    def on_robot_description(self, msg: String):
        try:
            model = arm_kinematics.UrdfModel.from_string(msg.data)
            names = model.movable_joint_names(self.target_link)
            if not names:
                raise ValueError(f"{self.target_link} 链上没有可动关节")
        except Exception as exc:
            self.get_logger().error(f"URDF 解析失败: {exc}")
            return

        self.model = model
        self.chain_names = names
        self.get_logger().info(
            f"IK 模型已加载: {len(names)} 个臂关节 {names}"
        )

    def handle_ik(self, request, response):
        response.error_code.val = MoveItErrorCodes.NO_IK_SOLUTION

        if self.model is None:
            self.get_logger().error("IK 模型尚未加载，忽略请求")
            return response

        pose = request.ik_request.pose_stamped
        target_link = request.ik_request.ik_link_name or self.target_link
        target_cm = [
            pose.pose.position.x * 100.0,
            pose.pose.position.y * 100.0,
            pose.pose.position.z * 100.0,
        ]

        seed = request.ik_request.robot_state.joint_state
        seed_deg = {
            name: math.degrees(float(position))
            for name, position in zip(seed.name, seed.position)
        }

        self.get_logger().info(
            "IK 请求: target=(%.2f, %.2f, %.2f) cm, link=%s, seed_joints=%d"
            % (*target_cm, target_link, len(seed.name))
        )

        try:
            solved_deg, error_cm = arm_kinematics.inverse_kinematics(
                self.model,
                target_cm,
                target_link=target_link,
                initial=seed_deg,
                max_iterations=self.max_iterations,
                tolerance_cm=self.tolerance_cm,
                restarts=self.restarts,
            )
        except Exception as exc:
            self.get_logger().error(f"IK 求解异常: {exc}")
            return response

        result_deg = {}
        for name in self.model.movable_joint_names(target_link):
            result_deg[name] = solved_deg.get(
                name, seed_deg.get(name, 0.0)
            )
        for name, value in seed_deg.items():
            result_deg.setdefault(name, value)

        response.solution.joint_state.header.stamp = (
            self.get_clock().now().to_msg()
        )
        response.solution.joint_state.header.frame_id = (
            pose.header.frame_id or self.model.base_link
        )
        response.solution.joint_state.name = list(result_deg.keys())
        response.solution.joint_state.position = [
            math.radians(value) for value in result_deg.values()
        ]

        success = error_cm <= self.tolerance_cm
        response.error_code.val = (
            MoveItErrorCodes.SUCCESS
            if success
            else MoveItErrorCodes.NO_IK_SOLUTION
        )

        self.get_logger().info(
            "IK 结果: error=%.3f cm, success=%s" % (error_cm, success)
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = IkSolverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
