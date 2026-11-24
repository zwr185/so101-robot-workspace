#!/usr/bin/env python3
"""MoveIt + Gazebo 抓取演示。

流程：
  1. 移动到 config/initial_positions.yaml 的初始位姿
  2. 打开夹爪
  3. 通过 /compute_ik 求出"夹爪 TCP 对准小方块中心、朝下"的关节角
  4. 规划并执行到抓取位姿
  5. 闭合夹爪
  6. manipulator 回到 SRDF 的 up 状态（夹爪保持 clamp）
  7. shoulder_pan 转到负方向极限
  8. 打开夹爪放置
  9. 恢复到 initial_positions

前提：启动时 Planning Scene 里不能包含 target_block（由 launch 传入
include_target_block:=false），否则夹爪无法规划到方块所在位置。
"""

import os
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

MANIPULATOR_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
SRDF_UP = [0.0, 0.0, 0.0, 0.0, 0.0]
# SRDF group_state "clamp" 是 -0.733，方块薄边只有 1.2cm，
# 闭合间隙需要更小才能夹紧；-0.80 在夹紧和避免过度穿插之间折中
CLAMP_GRIPPER = -0.80
# URDF 中 shoulder_pan 的下限是 -1.954291523766，留一点余量
PAN_NEGATIVE_LIMIT = -1.95
# 抓取点：小方块中心正上方。方块高 1.5cm，放在木板上表面 (world z=0.1)，
# 中心 world z=0.1075，base_link 在 world z=0.1，所以相对 base 为 z=0.0075。
GRASP_TCP_REL_BASE = (0.30, 0.0, 0.0075)
# 顶部抓取 + 绕 z 轴 90° 偏航：让两指沿方块 1.2cm 的薄边闭合
GRASP_LINK_QUAT = (0.0, 0.0, 0.7071068, 0.7071068)
# gripper_frame_link(TCP) 相对 gripper_link 的固定偏移（在 gripper_link 坐标系下）
TCP_OFFSET_LINK = (-0.0079, -0.000218, -0.0981274)
# R = Rz(90°) 时 R*(x, y, z) = (-y, x, z)
_OFFSET_ROTATED = (
    -TCP_OFFSET_LINK[1],
    TCP_OFFSET_LINK[0],
    TCP_OFFSET_LINK[2],
)
GRASP_LINK_POS = (
    GRASP_TCP_REL_BASE[0] - _OFFSET_ROTATED[0],
    GRASP_TCP_REL_BASE[1] - _OFFSET_ROTATED[1],
    GRASP_TCP_REL_BASE[2] - _OFFSET_ROTATED[2],
)


def load_initial_positions():
    share = get_package_share_directory("arm_gazebo_moveit2")
    path = os.path.join(share, "config", "initial_positions.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)["initial_positions"]


class GraspDemo(Node):
    def __init__(self):
        super().__init__("grasp_demo")
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
        )
        # move_group 的 /move_action 会比控制器先就绪，必须单独等控制器
        self.manipulator_client = ActionClient(
            self, FollowJointTrajectory,
            "/manipulator_controller/follow_joint_trajectory",
        )
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.latest_joints = None
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10
        )

    def _on_joint_states(self, msg: JointState):
        self.latest_joints = dict(zip(msg.name, msg.position))

    def _wait(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            return None
        return future.result()

    def wait_ready(self) -> bool:
        for name, client in (
            ("/move_action", self.move_client),
            ("/manipulator_controller/follow_joint_trajectory",
             self.manipulator_client),
            ("/gripper_controller/follow_joint_trajectory", self.gripper_client),
        ):
            if not client.wait_for_server(timeout_sec=60.0):
                self.get_logger().error(f"等待 action {name} 超时")
                return False
        if not self.ik_client.wait_for_service(timeout_sec=60.0):
            self.get_logger().error("等待 /compute_ik 服务超时")
            return False

        deadline = time.time() + 30.0
        while self.latest_joints is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.latest_joints is None:
            self.get_logger().error("没有收到 /joint_states")
            return False
        # 给控制器/JTC 一点时间完成激活
        time.sleep(2.0)
        return True

    def move_manipulator(self, positions, label: str) -> bool:
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = "manipulator"
        request.num_planning_attempts = 10
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5

        constraints = Constraints()
        for name, position in zip(MANIPULATOR_JOINTS, positions):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(position)
            joint.tolerance_above = 0.01
            joint.tolerance_below = 0.01
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False

        self.get_logger().info(f"[{label}] 规划并执行……")
        handle = self._wait(self.move_client.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            self.get_logger().error(f"[{label}] 目标被拒绝")
            return False

        wrapped = self._wait(handle.get_result_async(), 120.0)
        if wrapped is None:
            self.get_logger().error(f"[{label}] 执行超时")
            return False
        code = wrapped.result.error_code.val
        if code != 1:
            self.get_logger().error(f"[{label}] 失败，error_code={code}")
            return False
        self.get_logger().info(f"[{label}] 完成")
        return True

    def set_gripper(self, position: float, label: str) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["gripper"]
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start.sec = 2
        goal.trajectory.points.append(point)

        self.get_logger().info(f"[{label}] gripper -> {position:.3f}")
        handle = self._wait(self.gripper_client.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            self.get_logger().error(f"[{label}] 夹爪目标被拒绝")
            return False

        wrapped = self._wait(handle.get_result_async(), 30.0)
        if wrapped is None:
            self.get_logger().error(f"[{label}] 夹爪执行超时")
            return False
        if wrapped.result.error_code != 0:
            self.get_logger().error(
                f"[{label}] 夹爪失败：{wrapped.result.error_string}"
            )
            return False
        self.get_logger().info(f"[{label}] 完成")
        return True

    def solve_grasp_joints(self):
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = "manipulator"
        ik.robot_state.joint_state.name = MANIPULATOR_JOINTS
        ik.robot_state.joint_state.position = [
            float(self.latest_joints[name]) for name in MANIPULATOR_JOINTS
        ]
        ik.avoid_collisions = False
        ik.pose_stamped.header.frame_id = "base_link"
        ik.pose_stamped.pose.position.x = GRASP_LINK_POS[0]
        ik.pose_stamped.pose.position.y = GRASP_LINK_POS[1]
        ik.pose_stamped.pose.position.z = GRASP_LINK_POS[2]
        ik.pose_stamped.pose.orientation.x = GRASP_LINK_QUAT[0]
        ik.pose_stamped.pose.orientation.y = GRASP_LINK_QUAT[1]
        ik.pose_stamped.pose.orientation.z = GRASP_LINK_QUAT[2]
        ik.pose_stamped.pose.orientation.w = GRASP_LINK_QUAT[3]

        self.get_logger().info("求解抓取位姿的 IK……")
        response = self._wait(self.ik_client.call_async(request), 10.0)
        if response is None:
            self.get_logger().error("IK 服务超时")
            return None
        if response.error_code.val != 1:
            self.get_logger().error(
                f"IK 无解，error_code={response.error_code.val}"
            )
            return None

        solution = dict(
            zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            )
        )
        joints = [float(solution[name]) for name in MANIPULATOR_JOINTS]
        self.get_logger().info(f"IK 解：{[round(v, 3) for v in joints]}")
        return joints

    def run(self) -> bool:
        initial = load_initial_positions()
        initial_arm = [float(initial[name]) for name in MANIPULATOR_JOINTS]
        open_gripper = float(initial["gripper"])

        if not self.wait_ready():
            return False

        # 1. 从当前状态移动到 initial_positions
        if not self.move_manipulator(initial_arm, "1/9 初始位姿"):
            return False
        # 2. 打开夹爪
        if not self.set_gripper(open_gripper, "2/9 打开夹爪"):
            return False
        # 3. 求抓取位姿
        grasp = self.solve_grasp_joints()
        if grasp is None:
            return False
        # 4. 移动到小方块位置
        if not self.move_manipulator(grasp, "3/9 移动到小方块"):
            return False
        # 5. 闭合夹爪
        if not self.set_gripper(CLAMP_GRIPPER, "4/9 闭合夹爪"):
            return False
        # 6. manipulator 回到 up，夹爪保持 clamp
        if not self.move_manipulator(SRDF_UP, "5/9 回到 up"):
            return False
        # 7. shoulder_pan 转到负方向极限
        pan_limit = [PAN_NEGATIVE_LIMIT] + [0.0] * 4
        if not self.move_manipulator(pan_limit, "6/9 旋转到负限位"):
            return False
        # 8. 打开夹爪放置
        if not self.set_gripper(open_gripper, "7/9 放置"):
            return False
        # 9. 恢复到初始位姿
        if not self.move_manipulator(initial_arm, "8/9 恢复初始位姿"):
            return False
        self.get_logger().info("抓取演示完成")
        return True


def main():
    rclpy.init()
    node = GraspDemo()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
