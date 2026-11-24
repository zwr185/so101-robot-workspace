#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 GUI 的 /joint_states 转发成真机驱动的 cmd_angles。

数据链路：
  joint_state_gui  --/joint_states (rad, URDF 零点)-->  本节点
  本节点  --/cmd_angles (deg, 舵机 ID 1..6 顺序)-->  arm_joint_node

URDF 零点和真机 JSON 的 homing_offset 已经对齐，所以这里只需要：
  1. 按关节名取角度；
  2. 弧度转角度；
  3. 处理 wrist_roll 的方向约定（见 wrist_roll_sign 参数）。
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from arm_msg.msg import ArmJointAngles

JOINT_ORDER = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


class SimToRealControlNode(Node):
    def __init__(self):
        super().__init__("simtoreal_control")

        self.declare_parameter("input_topic", "joint_states")
        self.declare_parameter("output_topic", "cmd_angles")
        self.declare_parameter("rate_limit_hz", 30.0)
        self.declare_parameter("wrist_roll_sign", -1.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.rate_limit_hz = float(
            self.get_parameter("rate_limit_hz").value
        )
        self.wrist_roll_sign = float(
            self.get_parameter("wrist_roll_sign").value
        )

        self.publisher = self.create_publisher(
            ArmJointAngles, self.output_topic, 10
        )
        self.subscription = self.create_subscription(
            JointState, self.input_topic, self.on_joint_state, 10
        )
        self.subscription  # 避免 lint 警告

        self.last_publish = 0.0
        self.last_missing_warn = 0.0

        self.get_logger().info(
            f"遥操作转发已启动: {self.input_topic} -> {self.output_topic}；"
            f"wrist_roll_sign={self.wrist_roll_sign:+.1f}"
        )

    def on_joint_state(self, msg: JointState):
        now = time.monotonic()
        period = 1.0 / max(self.rate_limit_hz, 1.0)
        if now - self.last_publish < period:
            return

        values = {
            name: float(position)
            for name, position in zip(msg.name, msg.position)
        }
        missing = [name for name in JOINT_ORDER if name not in values]
        if missing:
            if now - self.last_missing_warn > 2.0:
                self.get_logger().warn(
                    f"/joint_states 缺少关节: {missing}"
                )
                self.last_missing_warn = now
            return

        angles = []
        for name in JOINT_ORDER:
            angle_deg = math.degrees(values[name])
            if name == "wrist_roll":
                angle_deg *= self.wrist_roll_sign
            angles.append(float(angle_deg))

        message = ArmJointAngles()
        message.angles = angles
        self.publisher.publish(message)
        self.last_publish = now


def main(args=None):
    rclpy.init(args=args)
    node = SimToRealControlNode()
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
