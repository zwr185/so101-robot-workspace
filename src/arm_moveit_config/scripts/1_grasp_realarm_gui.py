#!/usr/bin/env python3
"""真机 + MoveIt2 抓取界面（第 3 步固定 wrist_roll 的关节规划）。

相对 grasp_realarm_gui.py 的唯一流程改动：
  - 第 3 步固定 wrist_roll，用数值位置 IK 求前 4 个关节，
    让 gripper_frame_link 到达 GUI 给的末端位置，再交给 MoveIt 规划执行
  - 其余 1/2/4~8 步与原来一致

其他：
  - 所有输入坐标都是 base_link 坐标系
  - 上电即保持力矩，GUI 与 RViz 都可以直接执行轨迹
  - 速度/加速度缩放默认更保守（0.1）

流程：
  1 回 home        2 打开夹爪      3 到抓取点
  4 闭合夹爪       5 回 SRDF up    6 到放置点
  7 打开夹爪       8 回 home
"""

import math
import os
import sys
import threading
import time
import xml.etree.ElementTree as ET

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

MANIPULATOR_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
SRDF_UP = [0.0, 0.0, 0.0, 0.0, 0.0]

# 夹爪闭合值；张开值取自 initial_positions.yaml（SRDF "open"）
CLAMP_GRIPPER = -0.70
PAN_NEGATIVE_LIMIT = -1.95

# 抓取点（gripper_frame_link）相对木块中心的 X/Y/Z 偏移，可在界面里改
GRASP_X_OFFSET = 0.05
GRASP_Y_OFFSET = 0.03
GRASP_Z_OFFSET = 0.04

# 抓取时可调参数：wrist_roll 固定值、位置 IK 收敛阈值（m）
GRASP_WRIST_ROLL = 0.0
GRASP_POSITION_TOLERANCE = 0.004

# gripper_frame_link(TCP) 相对 gripper_link 的固定偏移（在 gripper_link 坐标系下）
TCP_OFFSET_LINK = (-0.0079, -0.000218, -0.0981274)

# 放置朝向沿用 shoulder_pan 负限位的 yaw，夹爪保持朝上
PLACE_LINK_QUAT = (
    0.0,
    0.0,
    math.sin(PAN_NEGATIVE_LIMIT / 2.0),
    math.cos(PAN_NEGATIVE_LIMIT / 2.0),
)
# 默认坐标（base_link 坐标），可在 config/grasp_realarm.yaml 里改
DEFAULT_BLOCK = (0.30, 0.00, 0.00)
DEFAULT_PLACE = (0.20, 0.20, 0.04)

# 放置点校验：base_link 平面为 0，向上 0.6 内、水平 0.55 内
PLACE_MIN_Z = 0.0
PLACE_MAX_Z = 0.6
PLACE_MAX_RADIUS = 0.55


def tcp_target_for_block(block_xyz, x_offset=GRASP_X_OFFSET,
                         y_offset=GRASP_Y_OFFSET,
                         z_offset=GRASP_Z_OFFSET):
    """GUI 输入的偏移直接作用在 gripper_frame_link 上。"""
    return (
        block_xyz[0] + x_offset,
        block_xyz[1] + y_offset,
        block_xyz[2] + z_offset,
    )


def solve3(mat, vec):
    """3x3 线性方程，Cramer 法则。"""
    def det(m):
        return (
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
        )

    d = det(mat)
    if abs(d) < 1e-12:
        return None
    out = []
    for col in range(3):
        m = [row[:] for row in mat]
        for row in range(3):
            m[row][col] = vec[row]
        out.append(det(m) / d)
    return out


def place_link_target(place_xyz):
    """根据放置点（gripper_frame_link 目标）的 base_link 坐标，算出 gripper_link 位姿。"""
    c = math.cos(PAN_NEGATIVE_LIMIT)
    s = math.sin(PAN_NEGATIVE_LIMIT)
    x, y, z = TCP_OFFSET_LINK
    offset_rotated = (
        c * x - s * y,
        s * x + c * y,
        z,
    )
    return (
        place_xyz[0] - offset_rotated[0],
        place_xyz[1] - offset_rotated[1],
        place_xyz[2] - offset_rotated[2],
    )


def validate_place(place_xyz):
    """检查放置点是否合法，返回 (ok, 原因)。"""
    x, y, z = place_xyz
    if not all(math.isfinite(v) for v in place_xyz):
        return False, "坐标包含非法数值"
    if z < PLACE_MIN_Z:
        return False, f"Z={z:.3f} 低于 base_link 平面 {PLACE_MIN_Z:.3f}"
    if z > PLACE_MAX_Z:
        return False, f"Z={z:.3f} 超出可放置高度"
    if math.hypot(x, y) > PLACE_MAX_RADIUS:
        return False, f"XY 距底座 {math.hypot(x, y):.3f}m，超出工作范围"
    return True, ""


def load_home_positions(log):
    """读取 SRDF home 对应的关节值（config/initial_positions.yaml）。"""
    try:
        share = get_package_share_directory("arm_moveit_config")
        path = os.path.join(share, "config", "initial_positions.yaml")
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)["initial_positions"]
    except (OSError, yaml.YAMLError, KeyError, TypeError) as exc:
        log(f"读取 initial_positions.yaml 失败：{exc}")
        return None


def load_default_point(section, default):
    """读取 config/grasp_realarm.yaml 里某一段的 x/y/z，缺失时用 default。"""
    try:
        share = get_package_share_directory("arm_moveit_config")
        path = os.path.join(share, "config", "grasp_realarm.yaml")
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        point = data.get(section, {})
        return (
            float(point.get("x", default[0])),
            float(point.get("y", default[1])),
            float(point.get("z", default[2])),
        )
    except (OSError, yaml.YAMLError, TypeError, ValueError):
        return default


def load_default_block():
    return load_default_point("block", DEFAULT_BLOCK)


def load_default_place():
    return load_default_point("place", DEFAULT_PLACE)


class GraspRos:
    """持有 ROS 节点，所有调用都由工作线程驱动。"""

    def __init__(self):
        self.node = Node("grasp_realarm_gui_1")
        self.move_client = ActionClient(self.node, MoveGroup, "/move_action")
        self.manipulator_client = ActionClient(
            self.node, FollowJointTrajectory,
            "/manipulator_controller/follow_joint_trajectory",
        )
        self.gripper_client = ActionClient(
            self.node, FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
        )
        self.ik_client = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_client = self.node.create_client(GetPositionFK, "/compute_fk")
        self.latest_joints = None
        # URDF 关节限位，用于在规划前给出 -26 的具体原因
        self.limits = None
        self.node.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10
        )

    def _on_joint_states(self, msg):
        self.latest_joints = dict(zip(msg.name, msg.position))

    def wait(self, future, timeout):
        deadline = time.time() + timeout
        while rclpy.ok() and not future.done() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return future.result() if future.done() else None

    def load_joint_limits(self):
        """从 /robot_description 解析 URDF 的关节限位。"""
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        holder = {}

        def _on_description(msg):
            holder.setdefault("urdf", msg.data)

        sub = self.node.create_subscription(
            String, "/robot_description", _on_description, qos
        )
        deadline = time.time() + 5.0
        while "urdf" not in holder and rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)

        urdf = holder.get("urdf")
        if not urdf:
            return None
        try:
            root = ET.fromstring(urdf)
        except ET.ParseError:
            return None

        limits = {}
        for joint in root.findall("joint"):
            if joint.get("type") not in ("revolute", "prismatic"):
                continue
            limit = joint.find("limit")
            if limit is None:
                continue
            lower, upper = limit.get("lower"), limit.get("upper")
            if lower is None or upper is None:
                continue
            limits[joint.get("name")] = (float(lower), float(upper))
        return limits or None

    def check_current_state(self, log):
        """规划前检查起始关节值，越界时说明 MoveIt 会返回 -26。"""
        if not self.limits:
            log("警告：未获取到 URDF 关节限位，跳过起始状态检查")
            return True

        problems = []
        for name in MANIPULATOR_JOINTS + ["gripper"]:
            if name not in self.latest_joints:
                problems.append(f"{name} 未在 /joint_states 中发布")
                continue
            value = float(self.latest_joints[name])
            lower, upper = self.limits.get(name, (-math.inf, math.inf))
            if not math.isfinite(value) or value < lower or value > upper:
                problems.append(
                    f"{name}={value:.3f} 超出 [{lower:.3f}, {upper:.3f}]"
                )

        if problems:
            log("起始状态非法（MoveIt 返回 -26 START_STATE_INVALID）：")
            for item in problems:
                log("  " + item)
            log("处理：力矩关闭时手动把上述关节摆回限位内，"
                "或修正 arm_hardware 的 zero_offsets / directions")
            return False

        log("起始状态检查通过")
        return True

    def wait_ready(self, log):
        for name, client in (
            ("/move_action", self.move_client),
            ("/manipulator_controller/follow_joint_trajectory",
             self.manipulator_client),
            ("/gripper_controller/follow_joint_trajectory", self.gripper_client),
        ):
            if not client.wait_for_server(timeout_sec=60.0):
                log(f"等待 action {name} 超时")
                return False
        if not self.ik_client.wait_for_service(timeout_sec=60.0):
            log("等待 /compute_ik 服务超时")
            return False
        if not self.fk_client.wait_for_service(timeout_sec=60.0):
            log("等待 /compute_fk 服务超时")
            return False
        deadline = time.time() + 30.0
        while self.latest_joints is None and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if self.latest_joints is None:
            log("没有收到 /joint_states")
            return False
        missing = [n for n in MANIPULATOR_JOINTS if n not in self.latest_joints]
        if missing:
            log(f"/joint_states 缺少机械臂关节：{missing}")
            return False
        if self.limits is None:
            self.limits = self.load_joint_limits()
        time.sleep(2.0)
        return True

    def _joint_snapshot(self):
        if not self.latest_joints:
            return {}
        return {
            name: float(self.latest_joints[name])
            for name in MANIPULATOR_JOINTS
            if name in self.latest_joints
        }

    def move_manipulator(self, positions, label, velocity_scale,
                         acceleration_scale, log):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = "manipulator"
        request.num_planning_attempts = 10
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = velocity_scale
        request.max_acceleration_scaling_factor = acceleration_scale

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

        before = self._joint_snapshot()
        log(f"[{label}] 规划并执行……")
        handle = self.wait(self.move_client.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            log(f"[{label}] 目标被拒绝")
            return False
        wrapped = self.wait(handle.get_result_async(), 120.0)
        if wrapped is None:
            log(f"[{label}] 执行超时")
            return False
        code = wrapped.result.error_code.val
        if code != 1:
            after = self._joint_snapshot()
            log(f"[{label}] 失败，error_code={code}（-4 = CONTROL_FAILED）")
            for name, target in zip(MANIPULATOR_JOINTS, positions):
                log(f"  {name}: 起点 {before.get(name, float('nan')):.3f} -> "
                    f"现在 {after.get(name, float('nan')):.3f} -> "
                    f"目标 {target:.3f}")
            traj = wrapped.result.planned_trajectory.joint_trajectory
            if traj.points:
                plan_start = dict(zip(traj.joint_names, traj.points[0].positions))
                diff = max(
                    (abs(before.get(n, 0.0) - plan_start.get(n, 0.0))
                     for n in MANIPULATOR_JOINTS),
                    default=0.0,
                )
                log(f"  规划起点与实际起点最大差异 {diff:.4f} rad")
            moved = {
                name: after.get(name, float("nan")) - before.get(name, float("nan"))
                for name in MANIPULATOR_JOINTS
            }
            if all(abs(delta) < 0.005 for delta in moved.values()):
                log("  结论：舵机完全没有动作 → 先确认力矩，"
                    "再看终端有没有“力矩使能失败”")
            else:
                reversed_joints = [
                    name for name, target in zip(MANIPULATOR_JOINTS, positions)
                    if abs(target - before.get(name, 0.0)) > 0.05
                    and moved[name] * (target - before.get(name, 0.0)) < 0
                ]
                if reversed_joints:
                    log(f"  结论：{reversed_joints} 往反方向动 → "
                        "directions 符号可能相反")
                elif max(
                    abs(after.get(name, 0.0) - target)
                    for name, target in zip(MANIPULATOR_JOINTS, positions)
                ) > 0.05:
                    log("  结论：有动作但没到目标 → "
                        "检查限位截断、速度或标定")
                else:
                    log("  结论：已接近目标，失败可能来自执行时间或容差")
            return False
        log(f"[{label}] 完成")
        return True

    def set_gripper(self, position, label, log):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["gripper"]
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start.sec = 2
        goal.trajectory.points.append(point)

        log(f"[{label}] gripper -> {position:.3f}")
        handle = self.wait(self.gripper_client.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            log(f"[{label}] 夹爪目标被拒绝")
            return False
        wrapped = self.wait(handle.get_result_async(), 30.0)
        if wrapped is None:
            log(f"[{label}] 夹爪执行超时")
            return False
        if wrapped.result.error_code != 0:
            log(f"[{label}] 夹爪失败：{wrapped.result.error_string}")
            return False
        log(f"[{label}] 完成")
        return True

    def _solve_ik(self, link_xyz, link_quat, log, label):
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = "manipulator"
        ik.robot_state.joint_state.name = MANIPULATOR_JOINTS
        ik.robot_state.joint_state.position = [
            float(self.latest_joints[name]) for name in MANIPULATOR_JOINTS
        ]
        ik.avoid_collisions = False
        ik.pose_stamped.header.frame_id = "base_link"
        ik.pose_stamped.pose.position.x = link_xyz[0]
        ik.pose_stamped.pose.position.y = link_xyz[1]
        ik.pose_stamped.pose.position.z = link_xyz[2]
        ik.pose_stamped.pose.orientation.x = link_quat[0]
        ik.pose_stamped.pose.orientation.y = link_quat[1]
        ik.pose_stamped.pose.orientation.z = link_quat[2]
        ik.pose_stamped.pose.orientation.w = link_quat[3]

        log(f"求解{label} IK，gripper_link 目标="
            f"({link_xyz[0]:.3f}, {link_xyz[1]:.3f}, {link_xyz[2]:.3f})……")
        response = self.wait(self.ik_client.call_async(request), 10.0)
        if response is None:
            log(f"{label} IK 服务超时")
            return None
        if response.error_code.val != 1:
            log(f"{label} IK 无解，error_code={response.error_code.val}")
            return None
        solution = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position,
        ))
        missing = [n for n in MANIPULATOR_JOINTS if n not in solution]
        if missing:
            log(f"{label} IK 结果缺少关节：{missing}")
            return None
        joints = [float(solution[name]) for name in MANIPULATOR_JOINTS]
        log(f"{label} IK 解：{[round(v, 3) for v in joints]}")
        return joints

    def _fk_tcp(self, positions):
        """用 /compute_fk 求 gripper_frame_link 的位置。"""
        request = GetPositionFK.Request()
        request.header.frame_id = "base_link"
        request.fk_link_names = ["gripper_frame_link"]
        request.robot_state.joint_state.name = MANIPULATOR_JOINTS + ["gripper"]
        request.robot_state.joint_state.position = (
            list(positions) + [float(self.latest_joints.get("gripper", 0.0))]
        )
        response = self.wait(self.fk_client.call_async(request), 3.0)
        if response is None or response.error_code.val != 1:
            return None
        p = response.pose_stamped[0].pose.position
        return (p.x, p.y, p.z)

    def solve_grasp_joints(self, target_tcp, log):
        """固定 wrist_roll，只解前 4 个关节，让 gripper_frame_link 到目标位置。

        用数值 Jacobian + 阻尼最小二乘迭代，初值是当前关节，
        收敛后把关节角交给 MoveIt 做轨迹规划。
        """
        if not self.latest_joints:
            log("没有关节状态，无法求解抓取关节")
            return None
        q = [float(self.latest_joints[n]) for n in MANIPULATOR_JOINTS]
        q[4] = GRASP_WRIST_ROLL
        lam = 1e-3
        eps = 1e-3
        dist = float("inf")
        for _ in range(30):
            p = self._fk_tcp(q)
            if p is None:
                log("FK 调用失败")
                return None
            err = [target_tcp[i] - p[i] for i in range(3)]
            dist = math.sqrt(sum(e * e for e in err))
            if dist < GRASP_POSITION_TOLERANCE:
                log(f"位置 IK 收敛：误差 {dist * 1000:.1f} mm，"
                    f"roll={q[4]:.3f}，解={[round(v, 3) for v in q]}")
                return q
            jac = [[0.0] * 4 for _ in range(3)]
            for j in range(4):
                qp = q[:]
                qp[j] += eps
                pj = self._fk_tcp(qp)
                if pj is None:
                    return None
                for k in range(3):
                    jac[k][j] = (pj[k] - p[k]) / eps
            ata = [
                [
                    sum(jac[r][c] * jac[s][c] for c in range(4))
                    + (lam if r == s else 0.0)
                    for s in range(3)
                ]
                for r in range(3)
            ]
            y = solve3(ata, err)
            if y is None:
                log("位置 IK 雅可比接近奇异，停止求解")
                return None
            for j in range(4):
                dq = sum(jac[k][j] * y[k] for k in range(3))
                q[j] += max(-0.15, min(0.15, dq))
                name = MANIPULATOR_JOINTS[j]
                if self.limits and name in self.limits:
                    lo, hi = self.limits[name]
                    q[j] = max(lo, min(hi, q[j]))
        log(f"位置 IK 未收敛，最后误差 {dist * 1000:.1f} mm；"
            "可调整 X/Y/Z 偏移或把目标点移近一些")
        return None

    def solve_place_joints(self, place_xyz, log):
        target = place_link_target(place_xyz)
        return self._solve_ik(target, PLACE_LINK_QUAT, log, "放置")


class Worker(QThread):
    log = pyqtSignal(str)
    finished_ok = pyqtSignal(bool)

    def __init__(self, ros, action, block_xyz, place_xyz, x_offset, y_offset,
                 z_offset, velocity_scale, acceleration_scale, stop_event,
                 parent=None):
        super().__init__(parent)
        self.ros = ros
        self.action = action
        self.block_xyz = block_xyz
        self.place_xyz = place_xyz
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.z_offset = z_offset
        self.velocity_scale = velocity_scale
        self.acceleration_scale = acceleration_scale
        self.stop_event = stop_event

    def _stopped(self):
        return self.stop_event.is_set()

    def run(self):
        try:
            ok = self._execute()
        except Exception as exc:  # noqa: BLE001
            self.log.emit(f"异常：{exc}")
            ok = False
        self.finished_ok.emit(ok)

    def _execute(self):
        if not self.ros.wait_ready(self.log.emit):
            return False
        if not self.ros.check_current_state(self.log.emit):
            return False

        home = load_home_positions(self.log.emit)
        if home is None:
            return False
        try:
            home_arm = [float(home[name]) for name in MANIPULATOR_JOINTS]
            open_gripper = float(home["gripper"])
        except (KeyError, TypeError, ValueError) as exc:
            self.log.emit(f"home 位姿缺少关节：{exc}")
            return False

        self.log.emit(f"home 位姿：{[round(v, 3) for v in home_arm]}")

        valid, reason = validate_place(self.place_xyz)
        if not valid:
            self.log.emit(f"放置位置无效：{reason}")
            return False

        if self.action == "reset":
            return self.ros.move_manipulator(
                home_arm, "复位到 home",
                self.velocity_scale, self.acceleration_scale, self.log.emit
            )

        tcp = tcp_target_for_block(
            self.block_xyz, self.x_offset, self.y_offset, self.z_offset
        )
        self.log.emit(
            f"抓取 TCP（gripper_frame_link）=({tcp[0]:.3f}, "
            f"{tcp[1]:.3f}, {tcp[2]:.3f})"
        )

        steps = [
            ("1/8 回 home", lambda: self.ros.move_manipulator(
                home_arm, "1/8 回 home",
                self.velocity_scale, self.acceleration_scale, self.log.emit)),
            ("2/8 打开夹爪", lambda: self.ros.set_gripper(
                open_gripper, "2/8 打开夹爪", self.log.emit)),
        ]
        for name, func in steps:
            if not func():
                self.log.emit("任务中断；处理后点“复位”回 home")
                return False
            if self._stopped():
                self.log.emit("已停止；处理后点“复位”回 home")
                return False

        # 第 3 步：固定 wrist_roll，用位置 IK 求前 4 个关节（关节目标），
        # 再交给 MoveIt 做轨迹规划
        grasp = self.ros.solve_grasp_joints(tcp, self.log.emit)
        if grasp is None:
            self.log.emit("抓取位置无解，流程未启动；可调整 X/Y/Z 偏移")
            return False
        place = self.ros.solve_place_joints(self.place_xyz, self.log.emit)
        if place is None:
            self.log.emit("放置位置不可达，抓取流程未启动")
            return False
        if self._stopped():
            self.log.emit("已停止")
            return False

        sequence = [
            ("3/8 移动到小木块", lambda: self.ros.move_manipulator(
                grasp, "3/8 移动到小木块",
                self.velocity_scale, self.acceleration_scale, self.log.emit)),
            ("4/8 闭合夹爪", lambda: self.ros.set_gripper(
                CLAMP_GRIPPER, "4/8 闭合夹爪", self.log.emit)),
            ("5/8 回到 up", lambda: self.ros.move_manipulator(
                SRDF_UP, "5/8 回到 up",
                self.velocity_scale, self.acceleration_scale, self.log.emit)),
            ("6/8 移动到放置位置", lambda: self.ros.move_manipulator(
                place, "6/8 移动到放置位置",
                self.velocity_scale, self.acceleration_scale, self.log.emit)),
            ("7/8 放置", lambda: self.ros.set_gripper(
                open_gripper, "7/8 放置", self.log.emit)),
            ("8/8 回 home", lambda: self.ros.move_manipulator(
                home_arm, "8/8 回 home",
                self.velocity_scale, self.acceleration_scale, self.log.emit)),
        ]
        for name, func in sequence:
            if not func():
                self.log.emit("任务中断；处理后点“复位”回 home")
                return False
            if self._stopped():
                self.log.emit("已停止；处理后点“复位”回 home")
                return False

        self.log.emit("抓取演示完成")
        return True


class MainWindow(QMainWindow):
    def __init__(self, ros):
        super().__init__()
        self.ros = ros
        self.worker = None
        self.stop_event = threading.Event()

        self.setWindowTitle("SO-101 真机抓取（wrist_roll 固定版）")
        central = QWidget()
        layout = QVBoxLayout(central)

        # 木块位置：默认值来自 config/grasp_realarm.yaml 的 block 段
        block_group = QGroupBox("小木块位置（base_link 坐标）")
        form = QFormLayout()
        default_block = load_default_block()
        self.block_x = self._spin(default_block[0], -1.0, 1.0, 0.01)
        self.block_y = self._spin(default_block[1], -1.0, 1.0, 0.01)
        self.block_z = self._spin(default_block[2], -0.2, 1.0, 0.005)
        self.x_offset = self._spin(GRASP_X_OFFSET, -0.3, 0.3, 0.005)
        self.y_offset = self._spin(GRASP_Y_OFFSET, -0.3, 0.3, 0.005)
        self.z_offset = self._spin(GRASP_Z_OFFSET, -0.2, 0.3, 0.005)
        form.addRow("X (m)", self.block_x)
        form.addRow("Y (m)", self.block_y)
        form.addRow("Z (m)", self.block_z)
        form.addRow("抓取点 X 偏移 (m)", self.x_offset)
        form.addRow("抓取点 Y 偏移 (m)", self.y_offset)
        form.addRow("抓取点 Z 偏移 (m)", self.z_offset)
        block_group.setLayout(form)
        layout.addWidget(block_group)

        # 放置位置（gripper_frame_link / TCP 的目标位置，base_link 坐标）
        place_group = QGroupBox(
            "放置位置（gripper_frame_link 目标，base_link 坐标）"
        )
        place_form = QFormLayout()
        default_place = load_default_place()
        self.place_x = self._spin(default_place[0], -0.6, 0.6, 0.01)
        self.place_y = self._spin(default_place[1], -0.6, 0.6, 0.01)
        self.place_z = self._spin(default_place[2], -0.2, 0.6, 0.005)
        for box in (self.place_x, self.place_y, self.place_z):
            box.setKeyboardTracking(False)
        place_form.addRow("X (m)", self.place_x)
        place_form.addRow("Y (m)", self.place_y)
        place_form.addRow("Z (m)", self.place_z)
        place_group.setLayout(place_form)
        layout.addWidget(place_group)

        # 运动参数
        motion_group = QGroupBox("运动参数")
        motion_form = QFormLayout()
        self.velocity = self._spin(0.1, 0.05, 1.0, 0.05)
        self.acceleration = self._spin(0.1, 0.05, 1.0, 0.05)
        motion_form.addRow("速度缩放", self.velocity)
        motion_form.addRow("加速度缩放", self.acceleration)
        motion_group.setLayout(motion_form)
        layout.addWidget(motion_group)

        hint = QLabel(
            "流程：1 home → 2 开爪 → 3 位置 IK 抓取（wrist_roll 固定 0）→ "
            "4 闭合 → 5 up → 6 放置点 → 7 松爪 → 8 home"
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)

        # 按钮
        buttons = QHBoxLayout()
        self.reset_button = QPushButton("复位")
        self.grasp_button = QPushButton("抓取")
        self.stop_button = QPushButton("停止抓取")
        self.stop_button.setEnabled(False)
        buttons.addWidget(self.reset_button)
        buttons.addWidget(self.grasp_button)
        buttons.addWidget(self.stop_button)
        layout.addLayout(buttons)

        layout.addWidget(QLabel("日志"))
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        layout.addWidget(self.log_view)

        self.setCentralWidget(central)
        self.resize(560, 520)

        self.reset_button.clicked.connect(lambda: self.start_worker("reset"))
        self.grasp_button.clicked.connect(lambda: self.start_worker("grasp"))
        self.stop_button.clicked.connect(self.request_stop)

    @staticmethod
    def _spin(value, minimum, maximum, step):
        box = QDoubleSpinBox()
        box.setDecimals(4)
        box.setRange(minimum, maximum)
        box.setSingleStep(step)
        box.setValue(value)
        return box

    def append_log(self, text):
        self.log_view.append(text)

    def start_worker(self, action):
        if self.worker is not None and self.worker.isRunning():
            self.append_log("当前任务尚未结束")
            return
        self.stop_event.clear()
        block = (
            self.block_x.value(),
            self.block_y.value(),
            self.block_z.value(),
        )
        place = (
            self.place_x.value(),
            self.place_y.value(),
            self.place_z.value(),
        )
        self.worker = Worker(
            self.ros,
            action,
            block,
            place,
            self.x_offset.value(),
            self.y_offset.value(),
            self.z_offset.value(),
            self.velocity.value(),
            self.acceleration.value(),
            self.stop_event,
        )
        self.worker.log.connect(self.append_log)
        self.worker.finished_ok.connect(self.on_finished)
        self.reset_button.setEnabled(False)
        self.grasp_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.append_log(
            f"开始：{action}，木块 {block}，放置 {place}，"
            f"X 偏移 {self.x_offset.value():.3f}，"
            f"Y 偏移 {self.y_offset.value():.3f}，"
            f"Z 偏移 {self.z_offset.value():.3f}"
        )
        self.worker.start()

    def request_stop(self):
        self.stop_event.set()
        self.append_log("已请求停止，将在当前动作结束后停止")

    def on_finished(self, ok):
        self.append_log("任务成功结束" if ok else "任务结束（未成功或已停止）")
        self.reset_button.setEnabled(True)
        self.grasp_button.setEnabled(True)
        self.stop_button.setEnabled(False)


def main():
    rclpy.init()
    ros = GraspRos()
    app = QApplication(sys.argv)
    window = MainWindow(ros)
    window.show()
    try:
        app.exec_()
    finally:
        ros.node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
