#!/usr/bin/env python3
"""带 PyQt5 界面的抓取演示。

界面功能：
  - 设置小木块中心的世界坐标（会同步到 Gazebo 里的 target_block）
  - 复位到 config/initial_positions.yaml 的初始位姿
  - 按 grasp_demo.py 的九步流程执行抓取
  - 停止抓取（在当前动作结束后停止）
  - 设置规划的速度 / 加速度缩放

ROS 交互全部在 QThread 里通过 spin_once 完成，GUI 线程不直接调用 ROS，
避免 Qt 与 rclpy 的线程冲突。
"""

import math
import os
import subprocess
import sys
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetPositionIK
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
CLAMP_GRIPPER = -0.80
PAN_NEGATIVE_LIMIT = -1.95
BASE_Z = 0.1  # base_link 在世界坐标中的 z

# 全局调试参数：抓取点（gripper_frame_link）相对木块世界坐标的偏移。
# TCP = (block_x, block_y + GRASP_Y_OFFSET, block_z + GRASP_Z_OFFSET)
# 也可以直接在界面上的“抓取点 Y/Z 偏移”里临时修改。
GRASP_Y_OFFSET = -0.02
GRASP_Z_OFFSET = 0.00

# 放置位姿：gripper_frame_link(TCP) 的目标位置（world 坐标）。
# 朝向沿用 shoulder_pan 负限位的 yaw（Rz(-1.95)），夹爪保持朝上。
PLACE_LINK_QUAT = (
    0.0,
    0.0,
    math.sin(PAN_NEGATIVE_LIMIT / 2.0),
    math.cos(PAN_NEGATIVE_LIMIT / 2.0),
)
# 默认放置点 = 原“旋转到负限位”姿态下 TCP 的世界坐标
DEFAULT_PLACE_WORLD = (-0.146, -0.367, 0.334)
BOARD_TOP_WORLD = 0.1
# 放置参数的持久化位置（实时保存 / 启动读取）
GUI_CONFIG_PATH = os.path.expanduser(
    "/home/zwr185/dev_ws/src/arm_gazebo_moveit2/config/grasp_gui.yaml"
)

# 顶部抓取 + 绕 z 轴 90° 偏航：两指沿方块 1.2cm 的薄边闭合
GRASP_LINK_QUAT = (0.0, 0.0, 0.7071068, 0.7071068)
TCP_OFFSET_LINK = (-0.0079, -0.000218, -0.0981274)


def load_initial_positions():
    share = get_package_share_directory("arm_gazebo_moveit2")
    path = os.path.join(share, "config", "initial_positions.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)["initial_positions"]


def link_target_for_block(block_xyz_world, y_offset=GRASP_Y_OFFSET,
                          z_offset=GRASP_Z_OFFSET):
    """根据木块中心的世界坐标和 Y/Z 偏移，算出 gripper_link 的目标位姿。"""
    tcp_world = (
        block_xyz_world[0],
        block_xyz_world[1] + y_offset,
        block_xyz_world[2] + z_offset,
    )
    tcp = (
        tcp_world[0],
        tcp_world[1],
        tcp_world[2] - BASE_Z,
    )
    # R = Rz(90°) 时 R*(x, y, z) = (-y, x, z)
    offset_rotated = (
        -TCP_OFFSET_LINK[1],
        TCP_OFFSET_LINK[0],
        TCP_OFFSET_LINK[2],
    )
    return (
        tcp[0] - offset_rotated[0],
        tcp[1] - offset_rotated[1],
        tcp[2] - offset_rotated[2],
    )


def place_link_target(place_xyz_world):
    """根据放置点的 world 坐标，算出 gripper_link 的目标位姿。

    放置朝向为 PLACE_LINK_QUAT（肩部负限位的 yaw），
    TCP(gripper_frame_link) 落在配置的放置点上。
    """
    tcp = (
        place_xyz_world[0],
        place_xyz_world[1],
        place_xyz_world[2] - BASE_Z,
    )
    c = math.cos(PAN_NEGATIVE_LIMIT)
    s = math.sin(PAN_NEGATIVE_LIMIT)
    x, y, z = TCP_OFFSET_LINK
    offset_rotated = (
        c * x - s * y,
        s * x + c * y,
        z,
    )
    return (
        tcp[0] - offset_rotated[0],
        tcp[1] - offset_rotated[1],
        tcp[2] - offset_rotated[2],
    )


def validate_place(place_xyz_world):
    """检查放置点是否合法，返回 (ok, 原因)。"""
    x, y, z = place_xyz_world
    if z < BOARD_TOP_WORLD:
        return False, f"Z={z:.3f} 低于木板上表面 {BOARD_TOP_WORLD:.3f}"
    if z > 0.6:
        return False, f"Z={z:.3f} 超出可放置高度"
    if math.hypot(x, y) > 0.55:
        return False, f"XY 距底座 {math.hypot(x, y):.3f}m，超出工作范围"
    if not all(math.isfinite(v) for v in place_xyz_world):
        return False, "坐标包含非法数值"
    return True, ""


def load_gui_config():
    try:
        with open(GUI_CONFIG_PATH, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        place = data.get("place", {})
        return (
            float(place.get("x", DEFAULT_PLACE_WORLD[0])),
            float(place.get("y", DEFAULT_PLACE_WORLD[1])),
            float(place.get("z", DEFAULT_PLACE_WORLD[2])),
        )
    except (OSError, yaml.YAMLError, TypeError, ValueError):
        return DEFAULT_PLACE_WORLD


def save_gui_config(place_xyz_world):
    try:
        os.makedirs(os.path.dirname(GUI_CONFIG_PATH), exist_ok=True)
        with open(GUI_CONFIG_PATH, "w", encoding="utf-8") as f:
            yaml.safe_dump(
                {
                    "place": {
                        "x": float(place_xyz_world[0]),
                        "y": float(place_xyz_world[1]),
                        "z": float(place_xyz_world[2]),
                    }
                },
                f,
                allow_unicode=True,
                sort_keys=True,
            )
        return True
    except OSError:
        return False


class GraspRos:
    """持有 ROS 节点，所有调用都由工作线程驱动。"""

    def __init__(self):
        self.node = Node("grasp_gui")
        self.move_client = ActionClient(self.node, MoveGroup, "/move_action")
        self.manipulator_client = ActionClient(
            self.node, FollowJointTrajectory,
            "/manipulator_controller/follow_joint_trajectory",
        )
        self.gripper_client = ActionClient(
            self.node, FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
        )
        self.ik_client = self.node.create_client(
            GetPositionIK, "/compute_ik"
        )
        self.latest_joints = None
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
        deadline = time.time() + 30.0
        while self.latest_joints is None and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if self.latest_joints is None:
            log("没有收到 /joint_states")
            return False
        time.sleep(2.0)
        return True

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
            log(f"[{label}] 失败，error_code={code}")
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

    def solve_grasp_joints(self, block_xyz_world, y_offset, z_offset, log):
        target = link_target_for_block(block_xyz_world, y_offset, z_offset)
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = "manipulator"
        ik.robot_state.joint_state.name = MANIPULATOR_JOINTS
        ik.robot_state.joint_state.position = [
            float(self.latest_joints[name]) for name in MANIPULATOR_JOINTS
        ]
        ik.avoid_collisions = False
        ik.pose_stamped.header.frame_id = "base_link"
        ik.pose_stamped.pose.position.x = target[0]
        ik.pose_stamped.pose.position.y = target[1]
        ik.pose_stamped.pose.position.z = target[2]
        ik.pose_stamped.pose.orientation.x = GRASP_LINK_QUAT[0]
        ik.pose_stamped.pose.orientation.y = GRASP_LINK_QUAT[1]
        ik.pose_stamped.pose.orientation.z = GRASP_LINK_QUAT[2]
        ik.pose_stamped.pose.orientation.w = GRASP_LINK_QUAT[3]

        log(f"求解 IK，木块=({block_xyz_world[0]:.3f}, "
            f"{block_xyz_world[1]:.3f}, {block_xyz_world[2]:.3f})，"
            f"Y 偏移={y_offset:.3f}，Z 偏移={z_offset:.3f}……")
        response = self.wait(self.ik_client.call_async(request), 10.0)
        if response is None:
            log("IK 服务超时")
            return None
        if response.error_code.val != 1:
            log(f"IK 无解，error_code={response.error_code.val}")
            return None
        solution = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position,
        ))
        joints = [float(solution[name]) for name in MANIPULATOR_JOINTS]
        log(f"IK 解：{[round(v, 3) for v in joints]}")
        return joints

    def solve_place_joints(self, place_xyz_world, log):
        target = place_link_target(place_xyz_world)
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = "manipulator"
        ik.robot_state.joint_state.name = MANIPULATOR_JOINTS
        ik.robot_state.joint_state.position = [
            float(self.latest_joints[name]) for name in MANIPULATOR_JOINTS
        ]
        ik.avoid_collisions = False
        ik.pose_stamped.header.frame_id = "base_link"
        ik.pose_stamped.pose.position.x = target[0]
        ik.pose_stamped.pose.position.y = target[1]
        ik.pose_stamped.pose.position.z = target[2]
        ik.pose_stamped.pose.orientation.x = PLACE_LINK_QUAT[0]
        ik.pose_stamped.pose.orientation.y = PLACE_LINK_QUAT[1]
        ik.pose_stamped.pose.orientation.z = PLACE_LINK_QUAT[2]
        ik.pose_stamped.pose.orientation.w = PLACE_LINK_QUAT[3]

        log(f"求解放置 IK，TCP 目标=({place_xyz_world[0]:.3f}, "
            f"{place_xyz_world[1]:.3f}, {place_xyz_world[2]:.3f})……")
        response = self.wait(self.ik_client.call_async(request), 10.0)
        if response is None:
            log("放置 IK 服务超时")
            return None
        if response.error_code.val != 1:
            log(f"放置 IK 无解，error_code={response.error_code.val}")
            return None
        solution = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position,
        ))
        joints = [float(solution[name]) for name in MANIPULATOR_JOINTS]
        log(f"放置 IK 解：{[round(v, 3) for v in joints]}")
        return joints

    @staticmethod
    def set_block_pose_in_gazebo(block_xyz_world):
        request = (
            'name: "target_block", '
            f'position: {{x: {block_xyz_world[0]}, '
            f'y: {block_xyz_world[1]}, z: {block_xyz_world[2]}}}, '
            "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
        )
        try:
            subprocess.run(
                [
                    "gz", "service",
                    "-s", "/world/so101_world/set_pose",
                    "--reqtype", "gz.msgs.Pose",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "3000",
                    "--req", request,
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=6,
            )
            return True
        except (OSError, subprocess.SubprocessError):
            return False


class Worker(QThread):
    log = pyqtSignal(str)
    finished_ok = pyqtSignal(bool)

    def __init__(self, ros, action, block_xyz, place_xyz, y_offset, z_offset,
                 velocity_scale, acceleration_scale, stop_event, parent=None):
        super().__init__(parent)
        self.ros = ros
        self.action = action
        self.block_xyz = block_xyz
        self.place_xyz = place_xyz
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
        initial = load_initial_positions()
        initial_arm = [float(initial[name]) for name in MANIPULATOR_JOINTS]
        open_gripper = float(initial["gripper"])

        valid, reason = validate_place(self.place_xyz)
        if not valid:
            self.log.emit(f"放置位置无效：{reason}")
            return False

        if not self.ros.wait_ready(self.log.emit):
            return False

        if self.action == "reset":
            return self.ros.move_manipulator(
                initial_arm, "复位到初始位姿",
                self.velocity_scale, self.acceleration_scale, self.log.emit
            )

        if not self.ros.set_block_pose_in_gazebo(self.block_xyz):
            self.log.emit("警告：设置 Gazebo 木块位姿失败，继续按输入坐标规划")

        if self._stopped():
            self.log.emit("已停止")
            return False

        steps = [
            ("1/9 初始位姿", lambda: self.ros.move_manipulator(
                initial_arm, "1/9 初始位姿",
                self.velocity_scale, self.acceleration_scale, self.log.emit)),
            ("2/9 打开夹爪", lambda: self.ros.set_gripper(
                open_gripper, "2/9 打开夹爪", self.log.emit)),
        ]
        for name, func in steps:
            if not func():
                return False
            if self._stopped():
                self.log.emit("已停止")
                return False

        grasp = self.ros.solve_grasp_joints(
            self.block_xyz, self.y_offset, self.z_offset, self.log.emit
        )
        if grasp is None:
            return False
        # 先验证放置点是否可达，不可达就不开始抓取
        place = self.ros.solve_place_joints(self.place_xyz, self.log.emit)
        if place is None:
            self.log.emit("放置位置不可达，抓取流程未启动")
            return False
        if self._stopped():
            self.log.emit("已停止")
            return False

        sequence = [
            ("3/9 移动到小木块", lambda: self.ros.move_manipulator(
                grasp, "3/9 移动到小木块",
                self.velocity_scale, self.acceleration_scale, self.log.emit)),
            ("4/9 闭合夹爪", lambda: self.ros.set_gripper(
                CLAMP_GRIPPER, "4/9 闭合夹爪", self.log.emit)),
            ("5/9 回到 up", lambda: self.ros.move_manipulator(
                SRDF_UP, "5/9 回到 up",
                self.velocity_scale, self.acceleration_scale, self.log.emit)),
            ("6/9 移动到放置位置", lambda: self.ros.move_manipulator(
                place, "6/9 移动到放置位置",
                self.velocity_scale, self.acceleration_scale, self.log.emit)),
            ("7/9 放置", lambda: self.ros.set_gripper(
                open_gripper, "7/9 放置", self.log.emit)),
            ("8/9 恢复初始位姿", lambda: self.ros.move_manipulator(
                initial_arm, "8/9 恢复初始位姿",
                self.velocity_scale, self.acceleration_scale, self.log.emit)),
        ]
        for name, func in sequence:
            if not func():
                return False
            if self._stopped():
                self.log.emit("已停止")
                return False

        self.log.emit("抓取演示完成")
        return True


class MainWindow(QMainWindow):
    def __init__(self, ros):
        super().__init__()
        self.ros = ros
        self.worker = None
        self.stop_event = threading.Event()

        self.setWindowTitle("SO-101 抓取演示")
        central = QWidget()
        layout = QVBoxLayout(central)

        # 木块位置
        block_group = QGroupBox("小木块位置（world 坐标）")
        form = QFormLayout()
        self.block_x = self._spin(0.30, -1.0, 1.0, 0.01)
        self.block_y = self._spin(0.0, -1.0, 1.0, 0.01)
        self.block_z = self._spin(0.115, 0.0, 1.0, 0.005)
        self.y_offset = self._spin(GRASP_Y_OFFSET, -0.3, 0.3, 0.005)
        self.z_offset = self._spin(GRASP_Z_OFFSET, -0.2, 0.3, 0.005)
        form.addRow("X (m)", self.block_x)
        form.addRow("Y (m)", self.block_y)
        form.addRow("Z (m)", self.block_z)
        form.addRow("抓取点 Y 偏移 (m)", self.y_offset)
        form.addRow("抓取点 Z 偏移 (m)", self.z_offset)
        block_group.setLayout(form)
        layout.addWidget(block_group)

        # 放置位置（gripper_frame_link / TCP 的目标位置，world 坐标）
        place_group = QGroupBox(
            "放置位置（gripper_frame_link 目标，world 坐标）"
        )
        place_form = QFormLayout()
        saved_place = load_gui_config()
        self.place_x = self._spin(saved_place[0], -0.6, 0.6, 0.01)
        self.place_y = self._spin(saved_place[1], -0.6, 0.6, 0.01)
        self.place_z = self._spin(saved_place[2], BOARD_TOP_WORLD, 0.6, 0.005)
        for box in (self.place_x, self.place_y, self.place_z):
            box.setKeyboardTracking(False)
            box.valueChanged.connect(self.save_place_config)
        place_form.addRow("X (m)", self.place_x)
        place_form.addRow("Y (m)", self.place_y)
        place_form.addRow("Z (m)", self.place_z)
        place_group.setLayout(place_form)
        layout.addWidget(place_group)

        # 运动参数
        motion_group = QGroupBox("运动参数")
        motion_form = QFormLayout()
        self.velocity = self._spin(0.5, 0.05, 1.0, 0.05)
        self.acceleration = self._spin(0.5, 0.05, 1.0, 0.05)
        motion_form.addRow("速度缩放", self.velocity)
        motion_form.addRow("加速度缩放", self.acceleration)
        motion_group.setLayout(motion_form)
        layout.addWidget(motion_group)

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
        self.resize(560, 480)

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

    def save_place_config(self):
        place = (
            self.place_x.value(),
            self.place_y.value(),
            self.place_z.value(),
        )
        if not save_gui_config(place):
            self.append_log("警告：放置位置保存失败")

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
