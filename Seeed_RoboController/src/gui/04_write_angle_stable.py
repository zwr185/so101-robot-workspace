#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
04 - 关节角度平滑运动
Write All Joint Angles (Smooth)

在 03_write_all_angle.py 的基础上，保留相同的角度输入界面，
但把「直接写目标位置」改为平滑轨迹：
  - 读取各关节当前位置作为起点
  - 用余弦缓动（起停速度为 0）在固定 1.2 秒内插值到目标角度
  - 定时下发中间目标位置，并按位移自动限制舵机速度/加速度
  - 支持「停止」按钮，避免大角度跳变产生冲击

用法 / Usage:
    # 在 src/gui 目录下运行
    python3 04_write_angle_stable.py
    python3 04_write_angle_stable.py ../json/my_awesome_follower_arm.json --port /dev/ttyACM0
    python3 04_write_angle_stable.py ../json/my_awesome_leader_arm.json --port /dev/ttyACM0

    # 或在仓库根目录运行
    python3 src/gui/04_write_angle_stable.py json/my_awesome_leader_arm.json --port /dev/ttyACM0
"""

import sys
import os
import re
import json
import math
import time
from pathlib import Path
from datetime import datetime

# 项目根目录加入路径
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, PROJECT_ROOT)

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QGroupBox, QDoubleSpinBox, QTextEdit, QMessageBox,
    QStatusBar, QLineEdit
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QTextCursor

try:
    from src.calibration_manager import JOINT_NAME_MAP
    from src.gui.theme_utils import setup_light_theme
except ImportError:
    JOINT_NAME_MAP = {}
    setup_light_theme = None

try:
    from scservo_sdk.port_handler import PortHandler
    from scservo_sdk.sms_sts import sms_sts
    from scservo_sdk.scservo_def import COMM_SUCCESS
except ImportError as e:
    print(f"❌ 无法导入 SCServo SDK: {e}")
    sys.exit(1)

# --- 校准文件 / Calibration file ---
DEFAULT_CALIBRATION_FILE = Path(
    "/home/zwr185/dev_ws/Seeed_RoboController/json/my_awesome_follower_arm.json"
)

# STS3215: 4096 计数对应 360°
COUNTS_PER_DEGREE = 4096.0 / 360.0
POSITION_MIN = 0
POSITION_MAX = 4095

DEFAULT_PORT = "/dev/ttyACM0"
BAUDRATE = 1000000

# --- 平滑运动参数 ---
TICK_MS = 30            # 轨迹下发周期（约 33Hz）
MOTION_DURATION = 1.2   # 固定运动时长（秒）
SERVO_ACC = 30          # 舵机加速度参数（0~254，越小越柔和）
MAX_SPEED = 2400        # 速度上限
MIN_SPEED = 60          # 速度下限
SPEED_MARGIN = 1.8      # 余弦缓动峰值速度约为平均速度的 1.5 倍，留出余量


def get_chinese_font(size=10, bold=False):
    """返回跨平台可用的中文字体"""
    import platform

    system = platform.system()
    if system == "Windows":
        family = "Microsoft YaHei"
    elif system == "Darwin":
        family = "PingFang SC"
    else:
        family = "Noto Sans CJK SC"

    font = QFont(family, size)
    font.setStyleHint(QFont.SansSerif)
    font.setBold(bold)
    return font


def load_calibration(path: Path) -> dict:
    """读取 LeRobot 校准文件，返回 {servo_id: joint_info}"""
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    joints = {}
    for joint_name, info in data.items():
        joints[int(info["id"])] = {
            "joint_name": joint_name,
            "homing_offset": int(info["homing_offset"]),
            "range_min": int(info["range_min"]),
            "range_max": int(info["range_max"]),
            "drive_mode": int(info.get("drive_mode", 0)),
        }
    return joints


def position_to_angle(joint: dict, position: float) -> float:
    """原始位置计数 -> 角度（相对校准零点，单位：度）"""
    sign = -1.0 if joint["drive_mode"] else 1.0
    return sign * (position - joint["homing_offset"]) / COUNTS_PER_DEGREE


def angle_to_position(joint: dict, angle: float) -> int:
    """角度（相对校准零点）-> 原始位置计数，并限制在校准范围内"""
    sign = -1.0 if joint["drive_mode"] else 1.0
    position = joint["homing_offset"] + sign * angle * COUNTS_PER_DEGREE
    position = int(round(position))
    position = max(joint["range_min"], min(joint["range_max"], position))
    return max(POSITION_MIN, min(POSITION_MAX, position))


def get_angle_limits(joint: dict) -> tuple:
    """由校准范围计算角度上下限"""
    a1 = position_to_angle(joint, joint["range_min"])
    a2 = position_to_angle(joint, joint["range_max"])
    low, high = sorted((a1, a2))
    return max(low, -180.0), min(high, 180.0)


class ServoController:
    """基于 SCServo SDK 的舵机控制（支持读取当前位置和速度/加速度写入）"""

    ADDR_TORQUE_ENABLE = 40

    def __init__(self, port=DEFAULT_PORT, baudrate=BAUDRATE, joints=None):
        self.port_name = port
        self.joints = joints or {}

        self.port_handler = PortHandler(port)
        if not self.port_handler.openPort():
            raise RuntimeError(f"无法打开串口 {port}")
        if not self.port_handler.setBaudRate(baudrate):
            self.port_handler.closePort()
            raise RuntimeError(f"无法设置波特率 {baudrate}")

        self.servo = sms_sts(self.port_handler)

    def close(self):
        try:
            self.port_handler.closePort()
        except Exception:
            pass

    def enable_torque(self, servo_id):
        result, _ = self.servo.write1ByteTxRx(
            servo_id, self.ADDR_TORQUE_ENABLE, 1
        )
        return result == COMM_SUCCESS

    def disable_torque(self, servo_id):
        result, _ = self.servo.write1ByteTxRx(
            servo_id, self.ADDR_TORQUE_ENABLE, 0
        )
        return result == COMM_SUCCESS

    def read_position(self, servo_id):
        """读取当前位置原始计数，失败返回 None"""
        position, result, _ = self.servo.ReadPos(servo_id)
        if result != COMM_SUCCESS:
            return None
        return position

    def write_goal(self, servo_id, position, speed, acc):
        """写入目标位置 + 速度 + 加速度（一次下发 7 字节）"""
        position = max(POSITION_MIN, min(POSITION_MAX, position))
        result, _ = self.servo.WritePosEx(servo_id, position, speed, acc)
        return result == COMM_SUCCESS


class StableAngleWindow(QMainWindow):
    """平滑运动主窗口"""

    def __init__(self, port_name: str, calibration_file: Path):
        super().__init__()
        self.port_name = port_name
        self.calibration_file = calibration_file
        self.joints = load_calibration(calibration_file)
        self.servo_ids = sorted(self.joints)
        self.angle_spins = {}

        # 轨迹状态
        self.motion_timer = QTimer(self)
        self.motion_timer.setInterval(TICK_MS)
        self.motion_timer.timeout.connect(self.on_motion_tick)
        self.motion_starts = {}
        self.motion_ends = {}
        self.motion_speeds = {}
        self.motion_duration = 0.0
        self.motion_t0 = 0.0
        self.motion_name = ""
        self.is_moving = False

        self.setWindowTitle("🤖 关节角度平滑控制 / Smooth Joint Control")
        self.setMinimumSize(900, 680)
        self.setFont(get_chinese_font(10))
        self.setStyleSheet("""
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
        """)

        self.controller = ServoController(
            port=port_name, baudrate=BAUDRATE, joints=self.joints
        )

        self.init_ui()
        self.enable_all_torque()

    # ---------------- UI ----------------

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        # 连接信息
        info_group = QGroupBox("🔌 连接信息")
        info_layout = QGridLayout()
        info_group.setLayout(info_layout)

        info_layout.addWidget(QLabel("串口:"), 0, 0)
        self.port_label = QLabel(self.port_name)
        self.port_label.setStyleSheet("color: #0d6efd; font-weight: bold;")
        info_layout.addWidget(self.port_label, 0, 1)

        info_layout.addWidget(QLabel("校准文件:"), 0, 2)
        self.file_label = QLabel(str(self.calibration_file))
        self.file_label.setStyleSheet("color: #495057;")
        info_layout.addWidget(self.file_label, 0, 3)

        info_layout.setColumnStretch(1, 1)
        info_layout.setColumnStretch(3, 3)
        layout.addWidget(info_group)

        # 角度输入
        angle_group = QGroupBox("🎯 关节角度输入（相对校准零点，单位：度）")
        grid = QGridLayout()
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(8)
        angle_group.setLayout(grid)

        headers = ["ID", "关节 / Joint", "目标角度", "校准范围"]
        for col, text in enumerate(headers):
            label = QLabel(text)
            label.setFont(get_chinese_font(10, bold=True))
            label.setStyleSheet("color: #495057;")
            grid.addWidget(label, 0, col)

        for row, servo_id in enumerate(self.servo_ids, start=1):
            joint = self.joints[servo_id]
            low, high = get_angle_limits(joint)

            id_label = QLabel(f"ID{servo_id}")
            id_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            grid.addWidget(id_label, row, 0)

            joint_name = JOINT_NAME_MAP.get(joint["joint_name"], joint["joint_name"])
            name_label = QLabel(joint_name)
            grid.addWidget(name_label, row, 1)

            spin = QDoubleSpinBox()
            spin.setRange(low, high)
            spin.setDecimals(1)
            spin.setSingleStep(1.0)
            spin.setSuffix(" °")
            spin.setValue(0.0)
            spin.setMinimumWidth(140)
            spin.setAlignment(Qt.AlignmentFlag.AlignCenter)
            grid.addWidget(spin, row, 2)
            self.angle_spins[servo_id] = spin

            range_label = QLabel(f"[{low:.1f}, {high:.1f}] °")
            range_label.setStyleSheet("color: #6c757d;")
            grid.addWidget(range_label, row, 3)

        grid.setColumnStretch(1, 2)
        layout.addWidget(angle_group)

        # 快捷输入 + 运动时长
        quick_layout = QHBoxLayout()
        quick_layout.addWidget(QLabel("⚡ 快速输入:"))

        self.quick_input = QLineEdit()
        self.quick_input.setPlaceholderText(
            "例如: 45,-50,-80,-45,50,0  （按 ID1~6 顺序，支持逗号/空格/括号）"
        )
        self.quick_input.setClearButtonEnabled(True)
        self.quick_input.returnPressed.connect(self.on_move_clicked)
        quick_layout.addWidget(self.quick_input, 1)

        quick_layout.addWidget(QLabel("运动时长:"))
        duration_label = QLabel(f"{MOTION_DURATION:.1f} s（固定）")
        duration_label.setStyleSheet("color: #0d6efd; font-weight: bold;")
        quick_layout.addWidget(duration_label)

        layout.addLayout(quick_layout)

        hint = QLabel(
            "💡 运动采用余弦缓动：起步和停止速度接近 0，中间过程自动限速，"
            f"固定时长 {MOTION_DURATION:.1f} 秒。"
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #6c757d; font-size: 11px;")
        layout.addWidget(hint)

        # 按钮
        button_layout = QHBoxLayout()

        self.move_btn = QPushButton("🚀 平滑运动")
        self.move_btn.setMinimumHeight(40)
        self.move_btn.clicked.connect(self.on_move_clicked)
        self.move_btn.setStyleSheet(
            "QPushButton { background-color: #0d6efd; color: white; border: none;"
            " padding: 8px 28px; border-radius: 5px; font-weight: bold; font-size: 14px; }"
            "QPushButton:hover { background-color: #0b5ed7; }"
            "QPushButton:disabled { background-color: #6c757d; }"
        )
        button_layout.addWidget(self.move_btn)

        self.reset_btn = QPushButton("🔄 复位")
        self.reset_btn.setMinimumHeight(40)
        self.reset_btn.setToolTip("全部关节平滑回到校准零点（0°）")
        self.reset_btn.clicked.connect(self.on_reset_clicked)
        self.reset_btn.setStyleSheet(
            "QPushButton { background-color: #6c757d; color: white; border: none;"
            " padding: 8px 28px; border-radius: 5px; font-weight: bold; font-size: 14px; }"
            "QPushButton:hover { background-color: #5c636a; }"
            "QPushButton:disabled { background-color: #adb5bd; }"
        )
        button_layout.addWidget(self.reset_btn)

        self.stop_btn = QPushButton("⏹ 停止")
        self.stop_btn.setMinimumHeight(40)
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.on_stop_clicked)
        self.stop_btn.setStyleSheet(
            "QPushButton { background-color: #dc3545; color: white; border: none;"
            " padding: 8px 28px; border-radius: 5px; font-weight: bold; font-size: 14px; }"
            "QPushButton:hover { background-color: #bb2d3b; }"
            "QPushButton:disabled { background-color: #adb5bd; }"
        )
        button_layout.addWidget(self.stop_btn)

        button_layout.addStretch()
        layout.addLayout(button_layout)

        # 日志
        log_group = QGroupBox("📋 操作日志")
        log_layout = QVBoxLayout()
        log_group.setLayout(log_layout)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMinimumHeight(140)
        self.log_text.setStyleSheet(
            "font-family: 'Consolas', 'Noto Sans Mono CJK SC', monospace; font-size: 12px;"
        )
        log_layout.addWidget(self.log_text)
        layout.addWidget(log_group, 1)

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("就绪")

    # ---------------- 日志/状态 ----------------

    def add_log(self, message: str):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        if self.log_text.document().blockCount() > 500:
            cursor = self.log_text.textCursor()
            cursor.movePosition(QTextCursor.Start)
            cursor.select(QTextCursor.LineUnderCursor)
            cursor.removeSelectedText()
        self.status_bar.showMessage(message)

    def set_moving(self, moving: bool):
        self.is_moving = moving
        self.move_btn.setEnabled(not moving)
        self.reset_btn.setEnabled(not moving)
        self.stop_btn.setEnabled(moving)
        self.quick_input.setEnabled(not moving)

    def enable_all_torque(self):
        print(f"Calibration file: {self.calibration_file}")
        for servo_id in self.servo_ids:
            self.controller.enable_torque(servo_id)
            time.sleep(0.05)

        self.add_log(f"✅ 已连接 {self.port_name}，并已使能全部舵机力矩")
        for servo_id in self.servo_ids:
            joint = self.joints[servo_id]
            low, high = get_angle_limits(joint)
            print(
                f"  ID{servo_id} {joint['joint_name']:<14} "
                f"zero={joint['homing_offset']:4d} "
                f"range=[{joint['range_min']}, {joint['range_max']}] "
                f"angle=[{low:7.1f}, {high:7.1f}]deg"
            )

    # ---------------- 平滑轨迹 ----------------

    def start_motion(self, targets: dict, action_name: str):
        """读取当前位置并启动平滑插值"""
        if self.is_moving:
            return

        starts = {}
        for servo_id in self.servo_ids:
            position = self.controller.read_position(servo_id)
            if position is None:
                self.add_log(f"❌ ID{servo_id} 当前位置读取失败，已取消运动")
                return
            starts[servo_id] = position

        ends = {
            servo_id: angle_to_position(self.joints[servo_id], targets[servo_id])
            for servo_id in self.servo_ids
        }
        duration = MOTION_DURATION

        speeds = {}
        for servo_id in self.servo_ids:
            delta = abs(ends[servo_id] - starts[servo_id])
            speed = int(delta / duration * SPEED_MARGIN) + 20
            speeds[servo_id] = max(MIN_SPEED, min(MAX_SPEED, speed))

        self.motion_starts = starts
        self.motion_ends = ends
        self.motion_speeds = speeds
        self.motion_duration = duration
        self.motion_t0 = time.monotonic()
        self.motion_name = action_name

        desc = "  ".join(
            f"ID{sid}: {position_to_angle(self.joints[sid], starts[sid]):.1f}"
            f"→{targets[sid]:.1f}°"
            for sid in self.servo_ids
        )
        self.add_log(f"🚀 开始{action_name}（{duration:.1f}s）: {desc}")
        self.set_moving(True)
        self.motion_timer.start()

    def on_motion_tick(self):
        """定时下发插值后的目标位置"""
        elapsed = time.monotonic() - self.motion_t0
        t = min(elapsed / self.motion_duration, 1.0)

        # 余弦缓动：t=0/1 时速度为 0，中段速度最大
        s = 0.5 - 0.5 * math.cos(math.pi * t)

        for servo_id in self.servo_ids:
            start = self.motion_starts[servo_id]
            end = self.motion_ends[servo_id]
            position = int(round(start + (end - start) * s))
            self.controller.write_goal(
                servo_id, position, self.motion_speeds[servo_id], SERVO_ACC
            )

        if t >= 1.0:
            self.motion_timer.stop()
            self.set_moving(False)
            self.add_log(f"✅ {self.motion_name}完成（用时 {elapsed:.1f}s）")
        else:
            self.status_bar.showMessage(
                f"{self.motion_name}中... {int(t * 100)}%"
            )

    def on_stop_clicked(self):
        if not self.is_moving:
            return
        self.motion_timer.stop()
        self.set_moving(False)
        self.add_log("⏹ 已停止，机械臂保持在当前中间位置")

    # ---------------- 输入处理 ----------------

    def parse_quick_input(self, text: str):
        """解析快捷输入，返回按 ID 顺序排列的角度列表；格式错误返回 None"""
        cleaned = (
            text.replace('（', '(').replace('）', ')')
                .replace('，', ',').replace('、', ',')
                .strip().strip('()[]{} ')
        )
        tokens = [token for token in re.split(r'[,;\s]+', cleaned) if token]

        try:
            values = [float(token.strip('°')) for token in tokens]
        except ValueError:
            QMessageBox.warning(
                self, "输入格式错误",
                f"无法解析角度: {text}\n示例: 45,-50,-80,-45,50,0"
            )
            return None

        if len(values) != len(self.servo_ids):
            QMessageBox.warning(
                self, "数量不匹配",
                f"需要 {len(self.servo_ids)} 个角度（ID{self.servo_ids[0]}~ID{self.servo_ids[-1]}），"
                f"当前输入 {len(values)} 个。"
            )
            return None

        out_of_range = []
        for servo_id, value in zip(self.servo_ids, values):
            spin = self.angle_spins[servo_id]
            if value < spin.minimum() or value > spin.maximum():
                out_of_range.append(
                    f"ID{servo_id}: {value:.1f}°（允许 {spin.minimum():.1f} ~ {spin.maximum():.1f}°）"
                )
        if out_of_range:
            QMessageBox.warning(
                self, "超出范围",
                "以下角度超出校准范围，未执行运动：\n" + "\n".join(out_of_range)
            )
            return None

        return values

    def on_move_clicked(self):
        if self.is_moving:
            return

        quick_text = self.quick_input.text().strip()
        if quick_text:
            values = self.parse_quick_input(quick_text)
            if values is None:
                return
            for servo_id, value in zip(self.servo_ids, values):
                self.angle_spins[servo_id].setValue(value)
            self.add_log(
                "⚡ 快速输入: " + ", ".join(f"{value:.1f}" for value in values)
            )
            self.quick_input.clear()

        targets = {
            servo_id: self.angle_spins[servo_id].value()
            for servo_id in self.servo_ids
        }
        self.start_motion(targets, "运动")

    def on_reset_clicked(self):
        if self.is_moving:
            return

        self.quick_input.clear()
        for servo_id in self.servo_ids:
            self.angle_spins[servo_id].setValue(0.0)

        self.start_motion(
            {servo_id: 0.0 for servo_id in self.servo_ids}, "复位"
        )

    # ---------------- 退出 ----------------

    def closeEvent(self, event):
        if self.is_moving:
            self.motion_timer.stop()
            self.set_moving(False)

        reply = QMessageBox.question(
            self, "退出确认",
            "退出后舵机将失能，机械臂可能因重力下落。\n确定退出吗？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            event.ignore()
            return

        try:
            for servo_id in self.servo_ids:
                self.controller.disable_torque(servo_id)
                time.sleep(0.05)
            self.controller.close()
        except Exception:
            pass
        event.accept()


def main():
    import argparse

    parser = argparse.ArgumentParser(description="关节角度平滑运动")
    parser.add_argument(
        'calibration', nargs='?', default=str(DEFAULT_CALIBRATION_FILE),
        help='LeRobot 校准文件路径'
    )
    parser.add_argument('--port', default=DEFAULT_PORT, help='串口，默认 /dev/ttyACM0')
    args, qt_args = parser.parse_known_args()
    sys.argv = [sys.argv[0]] + qt_args

    calibration_file = Path(args.calibration)
    if not calibration_file.is_file() and not calibration_file.is_absolute():
        # 相对路径在当前目录找不到时，按仓库 json/ 目录兜底
        candidate = Path(PROJECT_ROOT) / 'json' / calibration_file.name
        if candidate.is_file():
            calibration_file = candidate
    if not calibration_file.is_file():
        print(f"Fatal: calibration file not found: {args.calibration}")
        sys.exit(1)

    app = QApplication(sys.argv)
    if setup_light_theme:
        setup_light_theme(app)

    try:
        window = StableAngleWindow(args.port, calibration_file)
    except Exception as e:
        QMessageBox.critical(None, "启动失败", str(e))
        sys.exit(1)

    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
