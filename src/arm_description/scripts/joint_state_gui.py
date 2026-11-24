#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SO-101 自定义 joint_state_publisher GUI。

功能：
  * 订阅 /robot_description（TRANSIENT_LOCAL），解析所有可动关节；
  * 用角度滑块 / 数字框控制关节，发布 sensor_msgs/JointState 到 /joint_states；
  * 通过 tf2 实时显示 base_link -> gripper_frame_link 的平移，单位 cm；
  * 输入末端目标坐标，调用 IK 服务 /solve_ik 求解并更新关节滑块；
  * 采样可达工作空间，在等轴测视图中用灰点标出，并限制目标输入范围；
  * 关节零点已按 Seeed_RoboController 的真机标定写入 URDF，GUI 直接显示 URDF 角度。
  * IK 求解的是 gripper_frame_link 链上的 5 个臂关节，gripper 不参与末端位置。

运行：
  ros2 run arm_description joint_state_gui
或：
  ros2 launch arm_description display_custom_gui.launch.py
"""

import json
import math
import signal
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.time import Time
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformException

import arm_kinematics

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import (
    QBrush,
    QColor,
    QFont,
    QFontDatabase,
    QPainter,
    QPen,
)
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QSlider,
    QStatusBar,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

SLIDER_SCALE = 10.0  # 滑块 1 格 = 0.1 度 / 0.1 cm

# 让 UI 和 /joint_states 的顺序符合从底座到末端的直觉
PREFERRED_ORDER = {
    "shoulder_pan": 0,
    "shoulder_lift": 1,
    "elbow_flex": 2,
    "wrist_flex": 3,
    "wrist_roll": 4,
    "gripper": 5,
}

# URDF 已按真机标定，GUI 直接显示 URDF 角度。
# 如果以后还想在 UI 上叠加零点偏移，可在这里加 {"joint_name": offset_deg}。
JOINT_ZERO_OFFSETS = {}


@dataclass
class JointSpec:
    name: str
    kind: str
    lower: float  # 显示单位
    upper: float  # 显示单位
    unit: str
    offset: float = 0.0  # GUI 0 对应的 URDF 角度/位置偏移

    @property
    def is_angular(self) -> bool:
        return self.kind in ("revolute", "continuous")

    def display_to_ros(self, value: float) -> float:
        if self.is_angular:
            return math.radians(value + self.offset)
        return (value + self.offset) / 100.0

    def ros_to_display(self, value: float) -> float:
        if self.is_angular:
            return math.degrees(value) - self.offset
        return value * 100.0 - self.offset


def parse_joints(urdf_xml: str):
    """从展开后的 URDF 中提取可动关节及其限位。"""
    root = ET.fromstring(urdf_xml)
    specs = []

    for joint in root.findall("joint"):
        kind = joint.get("type", "")
        if kind not in ("revolute", "continuous", "prismatic"):
            continue
        if joint.find("mimic") is not None:
            continue

        name = joint.get("name")
        if not name:
            continue

        limit = joint.find("limit")

        if kind == "continuous":
            lower, upper = -180.0, 180.0
            unit = " °"
        elif kind == "prismatic":
            lo = float(limit.get("lower", "-0.1")) if limit is not None else -0.1
            hi = float(limit.get("upper", "0.1")) if limit is not None else 0.1
            lower, upper = lo * 100.0, hi * 100.0
            unit = " cm"
        else:
            lo = float(limit.get("lower", "0")) if limit is not None else 0.0
            hi = float(limit.get("upper", "0")) if limit is not None else 0.0
            lower, upper = math.degrees(lo), math.degrees(hi)
            unit = " °"

        if lower > upper:
            lower, upper = upper, lower
        if abs(upper - lower) < 1e-6:
            continue

        offset = JOINT_ZERO_OFFSETS.get(name, 0.0)
        lower -= offset
        upper -= offset

        specs.append(JointSpec(name, kind, lower, upper, unit, offset))

    specs.sort(key=lambda spec: PREFERRED_ORDER.get(spec.name, 999))
    return specs


class JointStateGuiNode(Node):
    """负责 ROS 侧的订阅、发布和 TF 查询。"""

    def __init__(self):
        super().__init__("arm_joint_state_gui")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tip_frame", "gripper_frame_link")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter(
            "calibration_file",
            "/home/zwr185/dev_ws/Seeed_RoboController/json/my_awesome_follower_arm.json",
        )
        self.declare_parameter("ik_service", "/solve_ik")

        self.base_frame = self.get_parameter("base_frame").value
        self.tip_frame = self.get_parameter("tip_frame").value
        rate = float(self.get_parameter("publish_rate").value)
        self.calibration_file = self.get_parameter("calibration_file").value
        self.ik_service = self.get_parameter("ik_service").value
        self.calibration = {}
        self.load_calibration()

        self.urdf_xml = None
        self.ik_client = self.create_client(GetPositionIK, self.ik_service)

        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String, "robot_description", self.on_robot_description, qos
        )
        self.publisher = self.create_publisher(JointState, "joint_states", 10)

        self.joint_order = []
        self.joint_positions = {}
        self.publish_enabled = True

        self.urdf_callback = None
        self.tf_callback = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(1.0 / max(rate, 1.0), self.publish_joint_states)
        self.create_timer(0.1, self.update_tf)

    def load_calibration(self):
        try:
            with open(self.calibration_file, "r", encoding="utf-8") as stream:
                data = json.load(stream)
            if not isinstance(data, dict):
                raise ValueError("JSON 顶层不是对象")
            self.calibration = data
            self.get_logger().info(
                f"已加载校准文件: {self.calibration_file} ({len(data)} 个关节)"
            )
            return True
        except Exception as exc:
            self.calibration = {}
            self.get_logger().error(f"校准文件加载失败: {exc}")
            return False

    def on_robot_description(self, msg: String):
        self.urdf_xml = msg.data
        try:
            specs = parse_joints(msg.data)
        except ET.ParseError as exc:
            self.get_logger().error(f"robot_description 解析失败: {exc}")
            return

        if not specs:
            self.get_logger().warning("robot_description 中没有找到可动关节")
            return

        self.joint_order = [spec.name for spec in specs]
        self.joint_positions = {}
        for spec in specs:
            self.joint_positions[spec.name] = 0.0

        if self.urdf_callback is not None:
            self.urdf_callback(specs)

        self.get_logger().info(
            f"已从 /robot_description 加载 {len(specs)} 个可动关节"
        )

    def publish_joint_states(self):
        if not self.publish_enabled or not self.joint_order:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_order)
        msg.position = [
            float(self.joint_positions.get(name, 0.0)) for name in self.joint_order
        ]
        self.publisher.publish(msg)

    def update_tf(self):
        if self.tf_callback is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.tip_frame, Time()
            )
        except TransformException:
            self.tf_callback(None)
            return

        t = transform.transform.translation
        self.tf_callback((t.x, t.y, t.z))


class PoseCanvas(QWidget):
    """把 base_link -> gripper_frame_link 的平移画成等轴测示意图。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.valid = False
        self.world = 0.45
        self.workspace_points = []
        self.target = None
        self.target_ok = None
        self.setMinimumHeight(260)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_pose(self, xyz):
        if xyz is None:
            if self.valid:
                self.valid = False
                self.update()
            return

        self.x, self.y, self.z = xyz
        self.valid = True
        self.update()

    def set_workspace_points(self, points):
        self.workspace_points = list(points)
        max_abs = 0.0
        for x, y, z in self.workspace_points:
            max_abs = max(max_abs, abs(x), abs(y), abs(z))
        self.world = max(0.45, max_abs * 1.15)
        self.update()

    def set_target(self, xyz, ok=None):
        self.target = xyz
        self.target_ok = ok
        self.update()

    @staticmethod
    def _project(x, y, z):
        # 简单等轴测投影：X 向右下，Y 向左下，Z 向上
        return (x - y) * 0.8660254, (x + y) * 0.5 - z

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#ffffff"))

        width = self.width()
        height = self.height()
        if width < 80 or height < 80:
            return

        world = self.world
        corners = []
        for x in (-world, world):
            for y in (-world, world):
                for z in (-world, world):
                    corners.append(self._project(x, y, z))

        min_u = min(c[0] for c in corners)
        max_u = max(c[0] for c in corners)
        min_v = min(c[1] for c in corners)
        max_v = max(c[1] for c in corners)

        margin = 36
        span_u = max(max_u - min_u, 1e-6)
        span_v = max(max_v - min_v, 1e-6)
        scale = min((width - 2 * margin) / span_u, (height - 2 * margin) / span_v)
        cx = width / 2.0
        cy = height / 2.0

        def to_screen(x, y, z):
            u, v = self._project(x, y, z)
            return (
                cx + (u - (min_u + max_u) / 2.0) * scale,
                cy - (v - (min_v + max_v) / 2.0) * scale,
            )

        ox, oy = to_screen(0.0, 0.0, 0.0)

        # 可达工作空间采样点
        if self.workspace_points:
            painter.setPen(QPen(QColor("#ced4da"), 1))
            for x, y, z in self.workspace_points:
                px, py = to_screen(x, y, z)
                painter.drawPoint(int(px), int(py))

        # 地面参考框
        plane = [
            (-world, -world, 0.0),
            (world, -world, 0.0),
            (world, world, 0.0),
            (-world, world, 0.0),
        ]
        points = [to_screen(*p) for p in plane]
        painter.setPen(QPen(QColor("#dee2e6"), 1, Qt.DashLine))
        for i in range(4):
            x1, y1 = points[i]
            x2, y2 = points[(i + 1) % 4]
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))

        # 坐标轴
        axis_len = 0.18
        axes = [
            ((axis_len, 0.0, 0.0), "#dc3545", "X"),
            ((0.0, axis_len, 0.0), "#198754", "Y"),
            ((0.0, 0.0, axis_len), "#0d6efd", "Z"),
        ]
        for end, color, label in axes:
            ex, ey = to_screen(*end)
            painter.setPen(QPen(QColor(color), 2))
            painter.drawLine(int(ox), int(oy), int(ex), int(ey))
            painter.setPen(QColor(color))
            painter.setFont(QFont("Noto Sans CJK SC", 9, QFont.Bold))
            painter.drawText(int(ex) + 4, int(ey) - 4, label)

        # 末端点
        if self.valid:
            px, py = to_screen(self.x, self.y, self.z)
            painter.setPen(QPen(QColor("#6c757d"), 1, Qt.DashLine))
            painter.drawLine(int(ox), int(oy), int(px), int(py))

            painter.setPen(QPen(QColor("#0d6efd"), 2))
            painter.setBrush(QBrush(QColor("#0d6efd")))
            painter.drawEllipse(int(px) - 5, int(py) - 5, 10, 10)

            painter.setPen(QColor("#0d6efd"))
            painter.setFont(QFont("Noto Sans CJK SC", 9))
            painter.drawText(int(px) + 8, int(py) - 8, "gripper")
        else:
            painter.setPen(QColor("#6c757d"))
            painter.setFont(QFont("Noto Sans CJK SC", 11))
            painter.drawText(self.rect(), Qt.AlignCenter, "等待 TF ...")

        # 目标点
        if self.target is not None:
            tx, ty, tz = self.target
            px, py = to_screen(tx, ty, tz)
            if self.target_ok is True:
                color = "#198754"
            elif self.target_ok is False:
                color = "#dc3545"
            else:
                color = "#fd7e14"

            painter.setPen(QPen(QColor(color), 2))
            painter.drawEllipse(int(px) - 6, int(py) - 6, 12, 12)
            painter.drawLine(int(px) - 9, int(py), int(px) + 9, int(py))
            painter.drawLine(int(px), int(py) - 9, int(px), int(py) + 9)
            painter.setPen(QColor(color))
            painter.setFont(QFont("Noto Sans CJK SC", 9))
            painter.drawText(int(px) + 10, int(py) + 18, "target")

        painter.setPen(QColor("#495057"))
        painter.setFont(QFont("Noto Sans CJK SC", 9))
        painter.drawText(int(ox) + 8, int(oy) + 16, "base_link")


class JointStateGuiWindow(QMainWindow):
    def __init__(self, node: JointStateGuiNode):
        super().__init__()
        self.node = node
        self.node.urdf_callback = self.on_urdf
        self.node.tf_callback = self.on_tf

        self.specs = []
        self.controls = {}
        self.tf_state = None
        self.kinematics = None
        self.workspace_bounds = None
        self.last_tf = None
        self.target_initialized = False
        self.ik_future = None
        self.ik_target_cm = None

        self.setWindowTitle("SO-101 关节调试面板")
        self.setMinimumSize(1120, 840)
        self.setFont(QFont("Noto Sans CJK SC", 10))

        self.build_ui()
        self.apply_style()

        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self.spin_ros)
        self.spin_timer.start(10)

        self.ik_timer = QTimer(self)
        self.ik_timer.timeout.connect(self.on_ik_result)

    def build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(10)

        status_group = QGroupBox("ROS 2 状态")
        status_layout = QGridLayout(status_group)
        status_layout.setHorizontalSpacing(12)
        status_layout.setVerticalSpacing(6)

        self.urdf_status = QLabel("等待 /robot_description ...")
        self.urdf_status.setStyleSheet("color: #fd7e14; font-weight: bold;")
        self.tf_status = QLabel("等待 TF ...")
        self.tf_status.setStyleSheet("color: #fd7e14; font-weight: bold;")

        status_layout.addWidget(QLabel("模型:"), 0, 0)
        status_layout.addWidget(self.urdf_status, 0, 1)
        status_layout.addWidget(QLabel("TF:"), 1, 0)
        status_layout.addWidget(self.tf_status, 1, 1)
        status_layout.setColumnStretch(1, 1)
        main_layout.addWidget(status_group)

        content_layout = QHBoxLayout()
        content_layout.setSpacing(10)

        self.joint_group = QGroupBox("关节控制")
        self.joint_grid = QGridLayout(self.joint_group)
        self.joint_grid.setHorizontalSpacing(14)
        self.joint_grid.setVerticalSpacing(8)
        content_layout.addWidget(self.joint_group, 3)

        self.side_tabs = QTabWidget()
        self.side_tabs.setDocumentMode(True)

        pose_tab = QWidget()
        pose_layout = QVBoxLayout(pose_tab)
        pose_layout.setContentsMargins(8, 8, 8, 8)
        pose_layout.setSpacing(8)

        self.pose_canvas = PoseCanvas()
        pose_layout.addWidget(self.pose_canvas, 1)

        readout = QGridLayout()
        readout.setHorizontalSpacing(10)
        readout.setVerticalSpacing(4)

        mono = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        mono.setPointSize(11)

        self.x_label = QLabel("--")
        self.y_label = QLabel("--")
        self.z_label = QLabel("--")
        self.distance_label = QLabel("--")
        for label in (
            self.x_label,
            self.y_label,
            self.z_label,
            self.distance_label,
        ):
            label.setFont(mono)
            label.setStyleSheet("color: #0d6efd; font-weight: bold;")

        readout.addWidget(QLabel("X:"), 0, 0)
        readout.addWidget(self.x_label, 0, 1)
        readout.addWidget(QLabel("Y:"), 1, 0)
        readout.addWidget(self.y_label, 1, 1)
        readout.addWidget(QLabel("Z:"), 2, 0)
        readout.addWidget(self.z_label, 2, 1)
        readout.addWidget(QLabel("距离:"), 3, 0)
        readout.addWidget(self.distance_label, 3, 1)
        readout.setColumnStretch(1, 1)
        pose_layout.addLayout(readout)

        target_group = QGroupBox("末端目标 (base_link, cm)")
        target_layout = QGridLayout(target_group)
        target_layout.setHorizontalSpacing(10)
        target_layout.setVerticalSpacing(6)

        self.target_x = QDoubleSpinBox()
        self.target_y = QDoubleSpinBox()
        self.target_z = QDoubleSpinBox()
        for spin in (self.target_x, self.target_y, self.target_z):
            spin.setRange(-100.0, 100.0)
            spin.setDecimals(1)
            spin.setSingleStep(0.5)
            spin.setSuffix(" cm")
            spin.setMinimumWidth(100)

        target_layout.addWidget(QLabel("X:"), 0, 0)
        target_layout.addWidget(self.target_x, 0, 1)
        target_layout.addWidget(QLabel("Y:"), 0, 2)
        target_layout.addWidget(self.target_y, 0, 3)
        target_layout.addWidget(QLabel("Z:"), 1, 0)
        target_layout.addWidget(self.target_z, 1, 1)

        self.use_current_button = QPushButton("用当前值")
        self.use_current_button.setMinimumHeight(30)
        self.use_current_button.clicked.connect(self.on_use_current)
        target_layout.addWidget(self.use_current_button, 1, 2)

        self.solve_button = QPushButton("求解并移动")
        self.solve_button.setMinimumHeight(30)
        self.solve_button.clicked.connect(self.on_solve_ik)
        target_layout.addWidget(self.solve_button, 1, 3)

        self.workspace_label = QLabel("等待 URDF ...")
        self.workspace_label.setWordWrap(True)
        self.workspace_label.setStyleSheet("color: #6c757d;")
        target_layout.addWidget(self.workspace_label, 2, 0, 1, 4)
        pose_layout.addWidget(target_group)
        self.side_tabs.addTab(pose_tab, "末端位置")

        calib_tab = QWidget()
        calib_layout = QVBoxLayout(calib_tab)
        calib_layout.setContentsMargins(8, 8, 8, 8)
        calib_layout.setSpacing(6)

        self.calib_table = QTableWidget(0, 6)
        self.calib_table.setHorizontalHeaderLabels(
            ["关节", "ID", "homing", "raw 范围", "drive", "URDF 限位"]
        )
        self.calib_table.verticalHeader().setVisible(False)
        self.calib_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.calib_table.setSelectionMode(QAbstractItemView.NoSelection)
        self.calib_table.setAlternatingRowColors(True)
        self.calib_table.setWordWrap(True)
        calib_header = self.calib_table.horizontalHeader()
        for column in range(5):
            calib_header.setSectionResizeMode(
                column, QHeaderView.ResizeToContents
            )
        calib_header.setSectionResizeMode(5, QHeaderView.Stretch)
        calib_layout.addWidget(self.calib_table, 1)

        self.calib_path_label = QLabel(self.node.calibration_file)
        self.calib_path_label.setWordWrap(True)
        self.calib_path_label.setStyleSheet("color: #6c757d;")
        calib_layout.addWidget(self.calib_path_label)

        self.reload_calib_button = QPushButton("重新读取校准")
        self.reload_calib_button.setMinimumHeight(32)
        self.reload_calib_button.clicked.connect(self.on_reload_calibration)
        calib_layout.addWidget(self.reload_calib_button)
        self.side_tabs.addTab(calib_tab, "校准参数")

        content_layout.addWidget(self.side_tabs, 2)
        main_layout.addLayout(content_layout, 1)

        button_layout = QHBoxLayout()

        self.zero_button = QPushButton("全部归零")
        self.zero_button.setMinimumHeight(36)
        self.zero_button.clicked.connect(self.on_zero)

        self.pause_button = QPushButton("暂停发布")
        self.pause_button.setMinimumHeight(36)
        self.pause_button.setCheckable(True)
        self.pause_button.toggled.connect(self.on_pause)

        button_layout.addWidget(self.zero_button)
        button_layout.addWidget(self.pause_button)

        hint = QLabel("拖动滑块或输入角度后自动发布到 /joint_states")
        hint.setStyleSheet("color: #6c757d;")
        button_layout.addWidget(hint)
        button_layout.addStretch()
        main_layout.addLayout(button_layout)

        log_group = QGroupBox("日志")
        log_layout = QVBoxLayout(log_group)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMinimumHeight(120)
        self.log_text.document().setMaximumBlockCount(500)
        log_layout.addWidget(self.log_text)
        main_layout.addWidget(log_group)

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("就绪")

    def apply_style(self):
        self.setStyleSheet(
            """
            QGroupBox {
                font-weight: bold;
                border: 1px solid #dee2e6;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 16px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 10px;
                padding: 0 6px 0 6px;
                color: #495057;
            }
            QPushButton {
                background-color: #0d6efd;
                color: white;
                border: none;
                padding: 7px 22px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #0b5ed7;
            }
            QPushButton:disabled {
                background-color: #adb5bd;
            }
            QPushButton:checked {
                background-color: #dc3545;
            }
            QLabel {
                color: #212529;
            }
            QTabWidget::pane {
                border: 1px solid #dee2e6;
                border-radius: 6px;
            }
            QTabBar::tab {
                padding: 6px 14px;
                color: #495057;
            }
            QTabBar::tab:selected {
                color: #0d6efd;
                font-weight: bold;
            }
            QTableWidget {
                gridline-color: #e9ecef;
                selection-background-color: #e7f1ff;
            }
            QHeaderView::section {
                background-color: #f8f9fa;
                color: #495057;
                padding: 4px;
                border: none;
                border-bottom: 1px solid #dee2e6;
            }
            """
        )

    def add_log(self, message: str):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        self.status_bar.showMessage(message)

    def spin_ros(self):
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except KeyboardInterrupt:
            self.spin_timer.stop()
            app = QApplication.instance()
            if app is not None:
                app.quit()
        except Exception as exc:  # 避免 GUI 因 ROS 退出异常直接崩掉
            self.spin_timer.stop()
            self.add_log(f"ROS spin 停止: {exc}")

    def on_urdf(self, specs):
        self.specs = specs
        self.rebuild_joint_controls(specs)
        self.update_calibration_table()

        try:
            self.kinematics = arm_kinematics.UrdfModel.from_string(
                self.node.urdf_xml
            )
            self.setup_workspace()
        except Exception as exc:
            self.kinematics = None
            self.workspace_label.setText(f"工作空间解析失败: {exc}")
            self.add_log(f"工作空间解析失败: {exc}")

        self.urdf_status.setText(f"已加载 {len(specs)} 个可动关节")
        self.urdf_status.setStyleSheet("color: #198754; font-weight: bold;")
        names = ", ".join(spec.name for spec in specs)
        self.add_log(f"URDF 已加载: {names}")

    def setup_workspace(self):
        if self.kinematics is None:
            return

        try:
            points_cm = arm_kinematics.sample_workspace(
                self.kinematics, "gripper_frame_link", samples=1200
            )
        except Exception as exc:
            self.add_log(f"工作空间采样失败: {exc}")
            return

        if len(points_cm) == 0:
            return

        lower = points_cm.min(axis=0) - 1.0
        upper = points_cm.max(axis=0) + 1.0
        self.workspace_bounds = (lower, upper)

        self.pose_canvas.set_workspace_points(
            [
                (float(x) / 100.0, float(y) / 100.0, float(z) / 100.0)
                for x, y, z in points_cm
            ]
        )

        for spin, low, high in zip(
            (self.target_x, self.target_y, self.target_z), lower, upper
        ):
            spin.setRange(float(low), float(high))

        self.workspace_label.setText(
            "可达采样包围盒 (cm): "
            f"X [{lower[0]:.1f}, {upper[0]:.1f}]  "
            f"Y [{lower[1]:.1f}, {upper[1]:.1f}]  "
            f"Z [{lower[2]:.1f}, {upper[2]:.1f}]\n"
            "包围盒内的点不一定都可解；IK 只求解 5 个臂关节，"
            "gripper 保持当前值。"
        )

        if self.last_tf is not None:
            self.set_target_inputs(self.last_tf)
        else:
            center = (lower + upper) / 2.0
            self.set_target_inputs(
                (float(center[0]), float(center[1]), float(center[2])),
                already_cm=True,
            )

    def set_target_inputs(self, xyz, already_cm=False):
        values = xyz if already_cm else tuple(
            value * 100.0 for value in xyz
        )
        for spin, value in zip(
            (self.target_x, self.target_y, self.target_z), values
        ):
            spin.setValue(float(value))

    def on_use_current(self):
        if self.last_tf is None:
            self.add_log("还没有收到 TF，无法使用当前值")
            return
        self.set_target_inputs(self.last_tf)
        self.target_initialized = True
        self.add_log("末端目标已设为当前实际位置")

    def on_solve_ik(self):
        if self.kinematics is None:
            self.add_log("URDF 尚未加载，无法求解 IK")
            return
        if not self.node.joint_order:
            self.add_log("关节尚未加载")
            return

        if not self.node.ik_client.service_is_ready():
            if not self.node.ik_client.wait_for_service(timeout_sec=1.0):
                self.add_log(f"IK 服务不可用: {self.node.ik_service}")
                return

        target_cm = [
            self.target_x.value(),
            self.target_y.value(),
            self.target_z.value(),
        ]

        if self.workspace_bounds is not None:
            lower, upper = self.workspace_bounds
            for index, value in enumerate(target_cm):
                if value < float(lower[index]) or value > float(upper[index]):
                    self.add_log("目标超出采样工作空间，请调整坐标")
                    return

        request = GetPositionIK.Request()
        request.ik_request.group_name = "manipulator"
        request.ik_request.ik_link_name = "gripper_frame_link"
        request.ik_request.pose_stamped.header.frame_id = "base_link"
        request.ik_request.pose_stamped.pose.position.x = target_cm[0] / 100.0
        request.ik_request.pose_stamped.pose.position.y = target_cm[1] / 100.0
        request.ik_request.pose_stamped.pose.position.z = target_cm[2] / 100.0
        request.ik_request.pose_stamped.pose.orientation.w = 1.0
        request.ik_request.robot_state.joint_state.name = list(
            self.node.joint_order
        )
        request.ik_request.robot_state.joint_state.position = [
            float(self.node.joint_positions.get(name, 0.0))
            for name in self.node.joint_order
        ]
        request.ik_request.timeout.sec = 2
        request.ik_request.timeout.nanosec = 0

        self.ik_target_cm = target_cm
        self.pose_canvas.set_target(
            (
                target_cm[0] / 100.0,
                target_cm[1] / 100.0,
                target_cm[2] / 100.0,
            ),
            ok=None,
        )
        self.ik_future = self.node.ik_client.call_async(request)
        self.ik_timer.start(20)
        self.add_log("IK 求解中: (%.1f, %.1f, %.1f) cm" % tuple(target_cm))

    def on_ik_result(self):
        if self.ik_future is None:
            self.ik_timer.stop()
            return
        if not self.ik_future.done():
            return

        self.ik_timer.stop()
        future = self.ik_future
        self.ik_future = None

        try:
            response = future.result()
        except Exception as exc:
            self.add_log(f"IK 服务调用失败: {exc}")
            return

        if (
            response is None
            or response.error_code.val != MoveItErrorCodes.SUCCESS
        ):
            self.add_log("IK 求解失败：目标可能不可达")
            if self.ik_target_cm is not None:
                self.pose_canvas.set_target(
                    (
                        self.ik_target_cm[0] / 100.0,
                        self.ik_target_cm[1] / 100.0,
                        self.ik_target_cm[2] / 100.0,
                    ),
                    ok=False,
                )
            return

        names = response.solution.joint_state.name
        positions = response.solution.joint_state.position

        for name, position in zip(names, positions):
            if name not in self.controls:
                continue
            spec, slider, spin = self.controls[name]
            display_value = spec.ros_to_display(float(position))

            spin.blockSignals(True)
            spin.setValue(display_value)
            spin.blockSignals(False)

            slider.blockSignals(True)
            slider.setValue(int(round(display_value * SLIDER_SCALE)))
            slider.blockSignals(False)

            self.node.joint_positions[name] = float(position)

        if self.ik_target_cm is None:
            self.add_log("IK 完成")
            return

        self.pose_canvas.set_target(
            (
                self.ik_target_cm[0] / 100.0,
                self.ik_target_cm[1] / 100.0,
                self.ik_target_cm[2] / 100.0,
            ),
            ok=True,
        )

        values_deg = {
            name: math.degrees(float(position))
            for name, position in zip(names, positions)
        }
        reached_cm = self.kinematics.forward_kinematics(
            values_deg, "gripper_frame_link"
        )[:3, 3]
        error_cm = math.sqrt(
            sum(
                (float(reached_cm[index]) - self.ik_target_cm[index]) ** 2
                for index in range(3)
            )
        )
        self.add_log(
            "IK 完成，位置误差 %.2f cm；5 个臂关节已更新，gripper 保持"
            % error_cm
        )

    def update_calibration_table(self):
        self.calib_table.setRowCount(0)

        for spec in self.specs:
            calibration = self.node.calibration.get(spec.name, {})
            row = self.calib_table.rowCount()
            self.calib_table.insertRow(row)

            values = [
                spec.name,
                str(calibration.get("id", "—")),
                str(calibration.get("homing_offset", "—")),
                f"{calibration.get('range_min', '—')}.."
                f"{calibration.get('range_max', '—')}",
                str(calibration.get("drive_mode", "—")),
                f"[{spec.lower:+.2f}, {spec.upper:+.2f}]{spec.unit.strip()}",
            ]

            for column, text in enumerate(values):
                item = QTableWidgetItem(text)
                if column == 0:
                    item.setTextAlignment(Qt.AlignLeft | Qt.AlignVCenter)
                    item.setToolTip(
                        json.dumps(calibration, ensure_ascii=False, indent=2)
                        if calibration
                        else "校准文件中没有该关节"
                    )
                else:
                    item.setTextAlignment(Qt.AlignCenter)
                self.calib_table.setItem(row, column, item)

        self.calib_table.resizeRowsToContents()

    def on_reload_calibration(self):
        if self.node.load_calibration():
            self.calib_path_label.setText(self.node.calibration_file)
            self.update_calibration_table()
            self.add_log("已重新读取真机校准文件")
        else:
            self.add_log("校准文件读取失败，请查看终端日志")

    def rebuild_joint_controls(self, specs):
        while self.joint_grid.count():
            item = self.joint_grid.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()

        self.controls = {}

        headers = ["关节", "滑块", "角度 / 位置", "范围"]
        for col, text in enumerate(headers):
            label = QLabel(text)
            font = label.font()
            font.setBold(True)
            label.setFont(font)
            label.setStyleSheet("color: #495057;")
            self.joint_grid.addWidget(label, 0, col)

        for row, spec in enumerate(specs, start=1):
            name_label = QLabel(spec.name)
            tooltip = f"类型: {spec.kind}"
            if abs(spec.offset) > 1e-9:
                tooltip += f"；GUI 0 = URDF {spec.offset:+.1f}{spec.unit}"
            name_label.setToolTip(tooltip)
            self.joint_grid.addWidget(name_label, row, 0)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(
                int(math.floor(spec.lower * SLIDER_SCALE)),
                int(math.ceil(spec.upper * SLIDER_SCALE)),
            )
            self.joint_grid.addWidget(slider, row, 1)

            spin = QDoubleSpinBox()
            spin.setRange(spec.lower, spec.upper)
            spin.setDecimals(1)
            spin.setSingleStep(1.0)
            spin.setSuffix(spec.unit)
            spin.setMinimumWidth(130)
            spin.setAlignment(Qt.AlignCenter)
            self.joint_grid.addWidget(spin, row, 2)

            range_label = QLabel(
                f"[{spec.lower:.1f}, {spec.upper:.1f}]{spec.unit}"
            )
            range_label.setStyleSheet("color: #6c757d;")
            self.joint_grid.addWidget(range_label, row, 3)

            initial = min(max(0.0, spec.lower), spec.upper)
            spin.blockSignals(True)
            spin.setValue(initial)
            spin.blockSignals(False)

            slider.blockSignals(True)
            slider.setValue(int(round(initial * SLIDER_SCALE)))
            slider.blockSignals(False)

            slider.valueChanged.connect(
                lambda value, name=spec.name: self.on_slider(name, value)
            )
            spin.valueChanged.connect(
                lambda value, name=spec.name: self.on_spin(name, value)
            )

            self.node.joint_positions[spec.name] = spec.display_to_ros(initial)
            self.controls[spec.name] = (spec, slider, spin)

        offset_specs = [spec for spec in specs if abs(spec.offset) > 1e-9]
        if offset_specs:
            notes = []
            for spec in offset_specs:
                unit = spec.unit.strip()
                notes.append(f"{spec.name} 0{unit} = {spec.offset:+.0f}{unit}")
            note_label = QLabel("零位偏移: " + "，".join(notes))
            note_label.setStyleSheet("color: #6c757d;")
            note_label.setWordWrap(True)
            self.joint_grid.addWidget(
                note_label, len(specs) + 1, 0, 1, 4
            )

        self.joint_grid.setColumnStretch(1, 1)
        self.joint_group.setTitle(f"关节控制（{len(specs)} 个可动关节）")

    def on_slider(self, name, value):
        if name not in self.controls:
            return
        spec, slider, spin = self.controls[name]
        display_value = value / SLIDER_SCALE

        spin.blockSignals(True)
        spin.setValue(display_value)
        spin.blockSignals(False)

        self.node.joint_positions[name] = spec.display_to_ros(display_value)

    def on_spin(self, name, value):
        if name not in self.controls:
            return
        spec, slider, spin = self.controls[name]

        slider.blockSignals(True)
        slider.setValue(int(round(value * SLIDER_SCALE)))
        slider.blockSignals(False)

        self.node.joint_positions[name] = spec.display_to_ros(value)

    def on_zero(self):
        for name, (spec, slider, spin) in self.controls.items():
            value = min(max(0.0, spec.lower), spec.upper)

            spin.blockSignals(True)
            spin.setValue(value)
            spin.blockSignals(False)

            slider.blockSignals(True)
            slider.setValue(int(round(value * SLIDER_SCALE)))
            slider.blockSignals(False)

            self.node.joint_positions[name] = spec.display_to_ros(value)

        self.add_log("所有关节已归零")

    def on_pause(self, checked):
        self.node.publish_enabled = not checked
        if checked:
            self.pause_button.setText("恢复发布")
            self.add_log("已暂停发布 /joint_states")
        else:
            self.pause_button.setText("暂停发布")
            self.add_log("已恢复发布 /joint_states")

    def on_tf(self, xyz):
        is_valid = xyz is not None

        if is_valid != self.tf_state:
            self.tf_state = is_valid
            if is_valid:
                self.tf_status.setText(
                    f"正常（{self.node.base_frame} → {self.node.tip_frame}）"
                )
                self.tf_status.setStyleSheet(
                    "color: #198754; font-weight: bold;"
                )
            else:
                self.tf_status.setText("等待 TF ...")
                self.tf_status.setStyleSheet(
                    "color: #fd7e14; font-weight: bold;"
                )

        if not is_valid:
            self.x_label.setText("--")
            self.y_label.setText("--")
            self.z_label.setText("--")
            self.distance_label.setText("--")
            self.pose_canvas.set_pose(None)
            return

        x, y, z = xyz
        self.x_label.setText(f"{x * 100.0:+.2f} cm")
        self.y_label.setText(f"{y * 100.0:+.2f} cm")
        self.z_label.setText(f"{z * 100.0:+.2f} cm")
        self.distance_label.setText(
            f"{math.sqrt(x * x + y * y + z * z) * 100.0:.2f} cm"
        )
        self.pose_canvas.set_pose((x, y, z))

        self.last_tf = (x, y, z)
        if not self.target_initialized:
            self.set_target_inputs(self.last_tf)
            self.target_initialized = True

    def closeEvent(self, event):
        self.spin_timer.stop()
        self.ik_timer.stop()
        super().closeEvent(event)


def main(args=None):
    rclpy.init(args=args)

    node = JointStateGuiNode()

    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())

    window = JointStateGuiWindow(node)
    window.show()

    try:
        exit_code = app.exec_()
    finally:
        window.spin_timer.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
