#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
03 - 手动输入全部关节角度并运动
Write All Joint Angles

参考 02_write_angle.py 的角度/位置换算与舵机控制方式：
  - 读取 LeRobot 校准文件（homing_offset / range_min / range_max）
  - 手动输入每个关节的目标角度（相对校准零点）
  - 支持在快捷输入框粘贴一组角度（如 45,-50,-80,-45,50,0），点击「运动」一次下发
  - 点击「运动」后机械臂运动到指定角度
  - 点击「复位」后全部关节回到校准零点（0°）

用法 / Usage:
    python src/gui/03_write_all_angle.py
    python src/gui/03_write_all_angle.py json/my_awesome_follower_arm.json --port /dev/ttyACM0
"""

import sys
import os
import re
import json
import time
from pathlib import Path
from datetime import datetime

# 项目根目录加入路径
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, PROJECT_ROOT)

import serial
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QGroupBox, QDoubleSpinBox, QTextEdit, QMessageBox,
    QStatusBar, QLineEdit
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont, QTextCursor

try:
    from src.calibration_manager import JOINT_NAME_MAP
    from src.gui.theme_utils import setup_light_theme
except ImportError:
    JOINT_NAME_MAP = {}
    setup_light_theme = None

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
    """直接通过串口写目标位置（与 02_write_angle.py 相同的方式）"""

    ADDR_GOAL_POSITION = 42
    ADDR_TORQUE_ENABLE = 40
    INST_WRITE = 3

    def __init__(self, port=DEFAULT_PORT, baudrate=BAUDRATE, timeout=0.1, joints=None):
        self.port_name = port
        self.serial_port = None
        self.joints = joints or {}

        try:
            self.serial_port = serial.Serial(port, baudrate=baudrate, timeout=timeout)
            time.sleep(0.1)
            self.serial_port.reset_input_buffer()
        except serial.SerialException as e:
            raise RuntimeError(f"无法打开串口 {port}: {e}")

    def close(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    @staticmethod
    def _calculate_checksum(data):
        return (~sum(data)) & 0xFF

    def _send_packet(self, servo_id, instruction, parameters=None):
        if not self.serial_port or not self.serial_port.is_open:
            return False
        if parameters is None:
            parameters = []

        length = len(parameters) + 2
        packet_core = [servo_id, length, instruction] + parameters
        checksum = self._calculate_checksum(packet_core)
        packet = bytes([0xFF, 0xFF] + packet_core + [checksum])

        try:
            self.serial_port.reset_input_buffer()
            self.serial_port.write(packet)
            self.serial_port.flush()
            return True
        except Exception:
            return False

    def _write_register(self, servo_id, address, value, size=2):
        params = [address]
        if size == 1:
            params.append(value & 0xFF)
        elif size == 2:
            params.extend([value & 0xFF, (value >> 8) & 0xFF])
        else:
            return False

        return self._send_packet(servo_id, self.INST_WRITE, params)

    def enable_torque(self, servo_id):
        return self._write_register(servo_id, self.ADDR_TORQUE_ENABLE, 1, size=1)

    def disable_torque(self, servo_id):
        return self._write_register(servo_id, self.ADDR_TORQUE_ENABLE, 0, size=1)

    def set_servo_angle(self, servo_id, angle):
        """把关节设置到指定角度（相对校准零点）"""
        joint = self.joints[servo_id]
        position = angle_to_position(joint, angle)
        ok = self._write_register(servo_id, self.ADDR_GOAL_POSITION, position, size=2)
        return ok, position


class WriteAllAngleWindow(QMainWindow):
    """手动输入角度并运动的主窗口"""

    def __init__(self, port_name: str, calibration_file: Path):
        super().__init__()
        self.port_name = port_name
        self.calibration_file = calibration_file
        self.joints = load_calibration(calibration_file)
        self.servo_ids = sorted(self.joints)
        self.angle_spins = {}

        self.setWindowTitle("🤖 关节角度控制 / Write All Joint Angles")
        self.setMinimumSize(860, 640)
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

        # 快捷输入
        quick_layout = QHBoxLayout()
        quick_layout.addWidget(QLabel("⚡ 快速输入:"))

        self.quick_input = QLineEdit()
        self.quick_input.setPlaceholderText(
            "例如: 45,-50,-80,-45,50,0  （按 ID1~6 顺序，支持逗号/空格/括号）"
        )
        self.quick_input.setClearButtonEnabled(True)
        self.quick_input.returnPressed.connect(self.on_move_clicked)
        quick_layout.addWidget(self.quick_input, 1)
        layout.addLayout(quick_layout)

        # 按钮
        button_layout = QHBoxLayout()

        self.move_btn = QPushButton("🚀 运动")
        self.move_btn.setMinimumHeight(40)
        self.move_btn.clicked.connect(self.on_move_clicked)
        self.move_btn.setStyleSheet(
            "QPushButton { background-color: #0d6efd; color: white; border: none;"
            " padding: 8px 28px; border-radius: 5px; font-weight: bold; font-size: 14px; }"
            "QPushButton:hover { background-color: #0b5ed7; }"
        )
        button_layout.addWidget(self.move_btn)

        self.reset_btn = QPushButton("🔄 复位")
        self.reset_btn.setMinimumHeight(40)
        self.reset_btn.setToolTip("全部关节回到校准零点（0°）")
        self.reset_btn.clicked.connect(self.on_reset_clicked)
        self.reset_btn.setStyleSheet(
            "QPushButton { background-color: #6c757d; color: white; border: none;"
            " padding: 8px 28px; border-radius: 5px; font-weight: bold; font-size: 14px; }"
            "QPushButton:hover { background-color: #5c636a; }"
        )
        button_layout.addWidget(self.reset_btn)

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

    # ---------------- 逻辑 ----------------

    def add_log(self, message: str):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        if self.log_text.document().blockCount() > 500:
            cursor = self.log_text.textCursor()
            cursor.movePosition(QTextCursor.Start)
            cursor.select(QTextCursor.LineUnderCursor)
            cursor.removeSelectedText()
        self.status_bar.showMessage(message)

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

    def move_to_angles(self, targets: dict, action_name: str):
        """把各关节运动到指定角度"""
        success = 0
        for servo_id in self.servo_ids:
            angle = targets[servo_id]
            joint = self.joints[servo_id]
            joint_name = JOINT_NAME_MAP.get(joint["joint_name"], joint["joint_name"])

            ok, position = self.controller.set_servo_angle(servo_id, angle)
            if ok:
                success += 1
                self.add_log(
                    f"✅ ID{servo_id} {joint_name}: {angle:6.1f}° -> raw {position}"
                )
            else:
                self.add_log(f"❌ ID{servo_id} {joint_name}: 发送失败")
            time.sleep(0.02)

        total = len(self.servo_ids)
        if success == total:
            self.add_log(f"🚀 {action_name}完成（{success}/{total} 个关节）")
        else:
            self.add_log(f"⚠️ {action_name}部分失败（{success}/{total} 个关节成功）")

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
        desc = "  ".join(
            f"ID{sid}={targets[sid]:.1f}°" for sid in self.servo_ids
        )
        print(f"[MOVE] {desc}")
        self.move_to_angles(targets, "运动")

    def on_reset_clicked(self):
        self.quick_input.clear()
        for servo_id in self.servo_ids:
            self.angle_spins[servo_id].setValue(0.0)
        self.add_log("🔄 复位：全部关节回到校准零点（0°）")
        self.move_to_angles(
            {servo_id: 0.0 for servo_id in self.servo_ids}, "复位"
        )

    # ---------------- 退出 ----------------

    def closeEvent(self, event):
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

    parser = argparse.ArgumentParser(description="手动输入关节角度并运动")
    parser.add_argument(
        'calibration', nargs='?', default=str(DEFAULT_CALIBRATION_FILE),
        help='LeRobot 校准文件路径'
    )
    parser.add_argument('--port', default=DEFAULT_PORT, help='串口，默认 /dev/ttyACM0')
    args, qt_args = parser.parse_known_args()
    sys.argv = [sys.argv[0]] + qt_args

    calibration_file = Path(args.calibration)
    if not calibration_file.is_file():
        print(f"Fatal: calibration file not found: {calibration_file}")
        sys.exit(1)

    app = QApplication(sys.argv)
    if setup_light_theme:
        setup_light_theme(app)

    try:
        window = WriteAllAngleWindow(args.port, calibration_file)
    except Exception as e:
        QMessageBox.critical(None, "启动失败", str(e))
        sys.exit(1)

    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
