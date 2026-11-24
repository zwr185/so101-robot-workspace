#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
舵机 PID 参数调节 GUI
Servo PID Tuning GUI

基于 PySide6 的可视化调参界面，复用 src/tools/servo_pid_tune.py 的寄存器读写逻辑：
    0x15 (21)  P 系数，出厂默认 32
    0x16 (22)  D 系数，出厂默认 32
    0x17 (23)  I 系数，出厂默认 0

用法 / Usage:
    python3 servo_pid_tune_gui.py
    python3 servo_pid_tune_gui.py --port /dev/ttyACM0
"""

import sys
import os
import time
import json
import platform
from datetime import datetime

# 项目根目录加入路径
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, PROJECT_ROOT)

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QComboBox, QGroupBox, QTableWidget, QTableWidgetItem,
    QHeaderView, QSpinBox, QTextEdit, QMessageBox, QStatusBar, QAbstractItemView,
    QFileDialog
)
from PySide6.QtCore import Qt, QThread, Signal, Slot, QObject, QMetaObject
from PySide6.QtGui import QFont, QColor, QTextCursor

try:
    from scservo_sdk.port_handler import PortHandler
    from scservo_sdk.sms_sts import sms_sts
except ImportError as e:
    print(f"❌ 错误: 无法导入 SCServo SDK: {e}")
    sys.exit(1)

try:
    from src.port_utils import get_available_ports
    from src.tools.servo_pid_tune import (
        BAUD_RATE, PID_VALUE_MAX, DEFAULT_SERVO_IDS,
        read_pid, write_pid, save_backup, scan_servos, BACKUP_DIR
    )
    from src.calibration_manager import JOINT_NAME_MAP
    from src.gui.theme_utils import setup_light_theme
except ImportError as e:
    print(f"❌ 错误: 无法导入项目模块: {e}")
    sys.exit(1)

# ID -> 关节名映射（SO-10x 标准配置）
ID_TO_JOINT = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}

COL_CHECK, COL_ID, COL_JOINT, COL_P, COL_D, COL_I, COL_STATUS = range(7)


def get_chinese_font(size=10, bold=False):
    """返回跨平台可用的中文字体"""
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


class PidWorker(QObject):
    """串口操作后台工作对象（运行在独立线程中）"""

    connection_changed = Signal(bool, str)   # 是否成功, 端口
    scan_finished = Signal(object)           # 发现的舵机 ID 列表
    pid_read = Signal(int, object)           # servo_id, {'p','d','i'}
    read_finished = Signal()
    write_result = Signal(int, bool, str)    # servo_id, 是否成功, 说明
    write_finished = Signal(bool, str)       # 是否全部成功, 备份文件路径
    log_message = Signal(str)
    error_occurred = Signal(str)

    def __init__(self):
        super().__init__()
        self.port_handler = None
        self.servo_handler = None
        self.port_name = ""

    def _close_port(self):
        if self.port_handler is not None:
            try:
                self.port_handler.closePort()
            except Exception:
                pass
        self.port_handler = None
        self.servo_handler = None

    @Slot(str)
    def connect_port(self, port_name: str):
        """连接舵机控制器"""
        try:
            self._close_port()

            port_handler = PortHandler(port_name)
            if not port_handler.openPort():
                self.error_occurred.emit(f"无法打开串口: {port_name}")
                self.connection_changed.emit(False, port_name)
                return
            if not port_handler.setBaudRate(BAUD_RATE):
                port_handler.closePort()
                self.error_occurred.emit("无法设置波特率")
                self.connection_changed.emit(False, port_name)
                return

            self.port_handler = port_handler
            self.servo_handler = sms_sts(port_handler)
            self.port_name = port_name
            self.log_message.emit(f"✅ 已连接 {port_name}（波特率 {BAUD_RATE}）")
            self.connection_changed.emit(True, port_name)

        except Exception as e:
            self._close_port()
            self.error_occurred.emit(f"连接失败: {e}")
            self.connection_changed.emit(False, port_name)

    @Slot()
    def disconnect_port(self):
        """断开连接"""
        if self.port_handler is not None:
            self._close_port()
            self.log_message.emit(f"🔌 已断开 {self.port_name}")
        self.port_name = ""
        self.connection_changed.emit(False, "")

    @Slot(object)
    def read_all(self, servo_ids: list):
        """扫描并读取所有目标舵机的 P/D/I"""
        if self.servo_handler is None:
            self.error_occurred.emit("未连接舵机控制器")
            self.read_finished.emit()
            return

        try:
            found = scan_servos(self.servo_handler)
            self.scan_finished.emit(found)
            self.log_message.emit(f"📡 发现舵机: {found}")

            for servo_id in servo_ids:
                if servo_id not in found:
                    continue
                values = read_pid(self.servo_handler, servo_id)
                if values is None:
                    self.log_message.emit(f"⚠️ ID{servo_id} P/D/I 读取失败")
                    continue
                self.pid_read.emit(servo_id, values)
        except Exception as e:
            self.error_occurred.emit(f"读取异常: {e}")
        finally:
            self.read_finished.emit()

    @Slot(object)
    def apply_pid(self, changes_by_id: dict):
        """备份原值后写入各舵机的 PID"""
        if self.servo_handler is None:
            self.error_occurred.emit("未连接舵机控制器")
            self.write_finished.emit(False, "")
            return

        try:
            # 先读取原值用于备份
            snapshot = {}
            for servo_id in changes_by_id:
                values = read_pid(self.servo_handler, servo_id)
                if values is not None:
                    snapshot[servo_id] = values

            if not snapshot:
                self.error_occurred.emit("没有可写入的舵机（原值读取失败）")
                self.write_finished.emit(False, "")
                return

            backup_path = save_backup(self.port_name, snapshot)
            self.log_message.emit(f"💾 原值已备份: {backup_path}")

            all_success = True
            for servo_id, changes in changes_by_id.items():
                if servo_id not in snapshot:
                    self.write_result.emit(servo_id, False, "原值读取失败，已跳过")
                    self.log_message.emit(f"❌ ID{servo_id}: 原值读取失败，已跳过")
                    all_success = False
                    continue

                success, message = write_pid(self.servo_handler, servo_id, changes)
                self.write_result.emit(servo_id, success, message)
                self.log_message.emit(
                    f"{'✅' if success else '❌'} ID{servo_id}: {message}"
                )
                all_success = all_success and success

                # 回读最新值刷新界面
                values = read_pid(self.servo_handler, servo_id)
                if values is not None:
                    self.pid_read.emit(servo_id, values)

                time.sleep(0.05)

            self.write_finished.emit(all_success, backup_path)

        except Exception as e:
            self.error_occurred.emit(f"写入异常: {e}")
            self.write_finished.emit(False, "")


class ServoPidTuneWindow(QMainWindow):
    """PID 调节主窗口"""

    connect_requested = Signal(str)
    disconnect_requested = Signal()
    read_requested = Signal(object)
    write_requested = Signal(object)

    def __init__(self, initial_port: str = None):
        super().__init__()
        self.connected = False
        self.busy = False
        self.writing = False
        self.initial_port = initial_port
        self.servo_loaded = set()

        self.p_spins = {}
        self.d_spins = {}
        self.i_spins = {}

        self.setWindowTitle("🎛️ 舵机 PID 参数调节工具 / Servo PID Tuning")
        self.setMinimumSize(1000, 700)
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

        self.init_ui()
        self.init_worker()
        self.refresh_ports()

        if self.initial_port:
            index = self.port_combo.findData(self.initial_port)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)
            self.start_connect()

    # ---------------- UI ----------------

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        # 连接设置
        conn_group = QGroupBox("🔌 连接设置")
        conn_layout = QHBoxLayout()
        conn_group.setLayout(conn_layout)

        conn_layout.addWidget(QLabel("串口:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(320)
        conn_layout.addWidget(self.port_combo)

        self.refresh_btn = QPushButton("🔄 刷新")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        conn_layout.addWidget(self.refresh_btn)

        self.connect_btn = QPushButton("🔗 连接")
        self.connect_btn.setStyleSheet(
            "QPushButton { background-color: #0d6efd; color: white; border: none;"
            " padding: 7px 18px; border-radius: 4px; font-weight: bold; }"
            "QPushButton:hover { background-color: #0b5ed7; }"
            "QPushButton:disabled { background-color: #6c757d; }"
        )
        self.connect_btn.clicked.connect(self.on_connect_clicked)
        conn_layout.addWidget(self.connect_btn)

        self.conn_label = QLabel("未连接")
        self.conn_label.setStyleSheet("color: #6c757d; font-weight: bold;")
        conn_layout.addWidget(self.conn_label)
        conn_layout.addStretch()
        layout.addWidget(conn_group)

        # PID 表格
        pid_group = QGroupBox("📖 PID 参数（P=0x15 / D=0x16 / I=0x17，取值 0~254）")
        pid_layout = QVBoxLayout()
        pid_group.setLayout(pid_layout)

        self.table = QTableWidget(len(DEFAULT_SERVO_IDS), 7)
        self.table.setHorizontalHeaderLabels(
            ["选择", "ID", "关节 / Joint", "P", "D", "I", "状态"]
        )
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setAlternatingRowColors(True)
        self.table.setMinimumHeight(290)

        header = self.table.horizontalHeader()
        header.setSectionResizeMode(COL_CHECK, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(COL_ID, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(COL_JOINT, QHeaderView.Stretch)
        header.setSectionResizeMode(COL_P, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(COL_D, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(COL_I, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(COL_STATUS, QHeaderView.Stretch)

        for row, servo_id in enumerate(DEFAULT_SERVO_IDS):
            self.table.setRowHeight(row, 38)

            check_item = QTableWidgetItem()
            check_item.setFlags(Qt.ItemFlag.ItemIsUserCheckable | Qt.ItemFlag.ItemIsEnabled)
            check_item.setCheckState(Qt.CheckState.Unchecked)
            self.table.setItem(row, COL_CHECK, check_item)

            id_item = QTableWidgetItem(str(servo_id))
            id_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            id_item.setFlags(Qt.ItemFlag.ItemIsEnabled)
            self.table.setItem(row, COL_ID, id_item)

            joint_name = ID_TO_JOINT.get(servo_id, "")
            joint_item = QTableWidgetItem(JOINT_NAME_MAP.get(joint_name, joint_name))
            joint_item.setFlags(Qt.ItemFlag.ItemIsEnabled)
            self.table.setItem(row, COL_JOINT, joint_item)

            for col, spins in ((COL_P, self.p_spins), (COL_D, self.d_spins), (COL_I, self.i_spins)):
                spin = QSpinBox()
                spin.setRange(0, PID_VALUE_MAX)
                spin.setAlignment(Qt.AlignmentFlag.AlignCenter)
                spin.setFixedWidth(90)
                spin.setEnabled(False)
                self.table.setCellWidget(row, col, spin)
                spins[servo_id] = spin

            status_item = QTableWidgetItem("未连接")
            status_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            status_item.setFlags(Qt.ItemFlag.ItemIsEnabled)
            self.table.setItem(row, COL_STATUS, status_item)

        pid_layout.addWidget(self.table)

        hint = QLabel(
            "💡 调参建议：一次只改一个参数（±8 左右）。静止高频抖→降 P；到位回摆→加 D；"
            "末端发软→加 P；I 一般保持 0。写入会自动备份原值到 json/ 目录。"
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #6c757d; font-size: 11px;")
        pid_layout.addWidget(hint)
        layout.addWidget(pid_group, 1)

        # 操作按钮
        action_layout = QHBoxLayout()

        self.read_btn = QPushButton("🔄 读取全部")
        self.read_btn.clicked.connect(self.on_read_clicked)
        action_layout.addWidget(self.read_btn)

        self.select_btn = QPushButton("☑️ 全选 / 取消全选")
        self.select_btn.clicked.connect(self.on_select_toggle)
        action_layout.addWidget(self.select_btn)

        action_layout.addStretch()

        self.restore_btn = QPushButton("↩️ 一键恢复备份")
        self.restore_btn.setToolTip("从 json/ 目录中的 PID 备份文件恢复参数")
        self.restore_btn.clicked.connect(self.on_restore_clicked)
        self.restore_btn.setStyleSheet(
            "QPushButton { background-color: #6f42c1; color: white; border: none;"
            " padding: 7px 18px; border-radius: 4px; font-weight: bold; }"
            "QPushButton:hover { background-color: #5a32a3; }"
            "QPushButton:disabled { background-color: #6c757d; }"
        )
        action_layout.addWidget(self.restore_btn)

        self.apply_selected_btn = QPushButton("✏️ 应用到选中")
        self.apply_selected_btn.clicked.connect(lambda: self.on_apply_clicked(True))
        self.apply_selected_btn.setStyleSheet(
            "QPushButton { background-color: #fd7e14; color: white; border: none;"
            " padding: 7px 18px; border-radius: 4px; font-weight: bold; }"
            "QPushButton:hover { background-color: #e56b0a; }"
            "QPushButton:disabled { background-color: #6c757d; }"
        )
        action_layout.addWidget(self.apply_selected_btn)

        self.apply_all_btn = QPushButton("✏️ 应用到全部")
        self.apply_all_btn.clicked.connect(lambda: self.on_apply_clicked(False))
        self.apply_all_btn.setStyleSheet(
            "QPushButton { background-color: #dc3545; color: white; border: none;"
            " padding: 7px 18px; border-radius: 4px; font-weight: bold; }"
            "QPushButton:hover { background-color: #bb2d3b; }"
            "QPushButton:disabled { background-color: #6c757d; }"
        )
        action_layout.addWidget(self.apply_all_btn)

        layout.addLayout(action_layout)

        # 日志
        log_group = QGroupBox("📋 操作日志")
        log_layout = QVBoxLayout()
        log_group.setLayout(log_layout)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMinimumHeight(110)
        self.log_text.setStyleSheet(
            "font-family: 'Consolas', 'Noto Sans Mono CJK SC', monospace; font-size: 12px;"
        )
        log_layout.addWidget(self.log_text)
        layout.addWidget(log_group)

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("就绪")

        self.update_ui_state()

    def init_worker(self):
        """创建后台工作线程"""
        self.worker = PidWorker()
        self.worker_thread = QThread(self)
        self.worker.moveToThread(self.worker_thread)

        self.connect_requested.connect(self.worker.connect_port)
        self.disconnect_requested.connect(self.worker.disconnect_port)
        self.read_requested.connect(self.worker.read_all)
        self.write_requested.connect(self.worker.apply_pid)

        self.worker.connection_changed.connect(self.on_connection_changed)
        self.worker.scan_finished.connect(self.on_scan_finished)
        self.worker.pid_read.connect(self.on_pid_read)
        self.worker.read_finished.connect(self.on_read_finished)
        self.worker.write_result.connect(self.on_write_result)
        self.worker.write_finished.connect(self.on_write_finished)
        self.worker.log_message.connect(self.add_log)
        self.worker.error_occurred.connect(self.on_error)

        self.worker_thread.start()

    # ---------------- 状态管理 ----------------

    def update_ui_state(self):
        self.refresh_btn.setEnabled(not self.busy)
        self.connect_btn.setEnabled(not self.busy)
        self.read_btn.setEnabled(self.connected and not self.busy)
        self.apply_selected_btn.setEnabled(self.connected and not self.busy and bool(self.servo_loaded))
        self.apply_all_btn.setEnabled(self.connected and not self.busy and bool(self.servo_loaded))
        self.restore_btn.setEnabled(self.connected and not self.busy and bool(self.servo_loaded))
        self.select_btn.setEnabled(self.connected and not self.busy)

        self.connect_btn.setText("🔌 断开" if self.connected else "🔗 连接")

    def set_busy(self, busy: bool):
        self.busy = busy
        self.update_ui_state()

    def add_log(self, message: str):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        if self.log_text.document().blockCount() > 500:
            cursor = self.log_text.textCursor()
            cursor.movePosition(QTextCursor.Start)
            cursor.select(QTextCursor.LineUnderCursor)
            cursor.removeSelectedText()
        self.status_bar.showMessage(message)

    def on_error(self, message: str):
        self.add_log(f"❌ {message}")

    def refresh_ports(self):
        """刷新串口列表"""
        current = self.port_combo.currentData()
        self.port_combo.clear()

        ports = get_available_ports()
        for port in ports:
            self.port_combo.addItem(f"{port.device}  ({port.description})", port.device)

        if not ports:
            self.port_combo.addItem("未发现可用串口", None)

        if current:
            index = self.port_combo.findData(current)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

        self.add_log(f"🔄 可用串口: {[p.device for p in ports] if ports else '无'}")

    # ---------------- 按钮回调 ----------------

    def start_connect(self):
        port_name = self.port_combo.currentData()
        if not port_name:
            QMessageBox.warning(self, "提示", "未检测到可用串口，请检查 USB 连接后刷新。")
            return
        self.set_busy(True)
        self.add_log(f"🔗 正在连接 {port_name}...")
        self.connect_requested.emit(port_name)

    def on_connect_clicked(self):
        if self.connected:
            self.set_busy(True)
            self.disconnect_requested.emit()
        else:
            self.start_connect()

    def on_read_clicked(self):
        self.writing = False
        self.set_busy(True)
        self.servo_loaded.clear()
        for row in range(self.table.rowCount()):
            self.table.item(row, COL_STATUS).setText("读取中...")
        self.read_requested.emit(list(DEFAULT_SERVO_IDS))

    def on_select_toggle(self):
        all_checked = all(
            self.table.item(row, COL_CHECK).checkState() == Qt.CheckState.Checked
            for row in range(self.table.rowCount())
        )
        state = Qt.CheckState.Unchecked if all_checked else Qt.CheckState.Checked
        for row, servo_id in enumerate(DEFAULT_SERVO_IDS):
            item = self.table.item(row, COL_CHECK)
            if servo_id in self.servo_loaded:
                item.setCheckState(state)
            else:
                item.setCheckState(Qt.CheckState.Unchecked)

    def on_apply_clicked(self, only_selected: bool):
        changes_by_id = {}
        for row, servo_id in enumerate(DEFAULT_SERVO_IDS):
            if servo_id not in self.servo_loaded:
                continue
            if only_selected and self.table.item(row, COL_CHECK).checkState() != Qt.CheckState.Checked:
                continue
            changes_by_id[servo_id] = {
                'p': self.p_spins[servo_id].value(),
                'd': self.d_spins[servo_id].value(),
                'i': self.i_spins[servo_id].value(),
            }

        if not changes_by_id:
            QMessageBox.information(
                self, "提示",
                "没有可写入的舵机。\n请先读取成功，并勾选要写入的 ID。" if only_selected
                else "没有可写入的舵机，请先读取成功。"
            )
            return

        target_desc = ", ".join(f"ID{sid}" for sid in sorted(changes_by_id))
        reply = QMessageBox.warning(
            self, "确认写入",
            f"即将写入: {target_desc}\n\n"
            "⚠️ 写入 EEPROM 时会暂时断力矩，机械臂可能因重力下落，请先托住机械臂！\n"
            "原值会自动备份到 json/ 目录。\n\n确定继续？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            return

        self.start_write(changes_by_id)

    def on_restore_clicked(self):
        """从备份文件恢复 PID"""
        latest = self.find_latest_backup()
        if not latest:
            QMessageBox.information(
                self, "提示",
                f"未找到备份文件。\n备份目录: {BACKUP_DIR}"
            )
            return

        path, _ = QFileDialog.getOpenFileName(
            self, "选择 PID 备份文件", latest,
            "PID 备份 (*.json);;所有文件 (*)"
        )
        if not path:
            return

        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            servos = data.get('servos', {})
        except Exception as e:
            QMessageBox.critical(self, "错误", f"备份文件读取失败:\n{e}")
            self.add_log(f"❌ 备份文件读取失败: {e}")
            return

        changes_by_id = {}
        for key, values in servos.items():
            try:
                servo_id = int(key)
            except (TypeError, ValueError):
                continue
            if servo_id not in DEFAULT_SERVO_IDS or servo_id not in self.servo_loaded:
                continue
            if not isinstance(values, dict):
                continue
            try:
                p, d, i = int(values['p']), int(values['d']), int(values['i'])
            except (KeyError, TypeError, ValueError):
                continue
            if not all(0 <= value <= PID_VALUE_MAX for value in (p, d, i)):
                continue
            changes_by_id[servo_id] = {'p': p, 'd': d, 'i': i}

        if not changes_by_id:
            QMessageBox.information(
                self, "提示",
                "备份中没有当前已读取成功的舵机参数。\n请先读取全部后再恢复。"
            )
            return

        detail = "\n".join(
            f"  ID{servo_id}: P={values['p']}  D={values['d']}  I={values['i']}"
            for servo_id, values in sorted(changes_by_id.items())
        )
        reply = QMessageBox.warning(
            self, "确认恢复",
            f"备份文件: {os.path.basename(path)}\n"
            f"备份时间: {data.get('timestamp', '未知')}\n\n{detail}\n\n"
            "⚠️ 恢复会覆盖当前 PID，写入时会短暂断力矩，请先托住机械臂！\n"
            "当前参数会自动再备份一次。\n\n确定恢复？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            return

        self.add_log(f"↩️ 从备份恢复: {path}")
        self.start_write(changes_by_id)

    def find_latest_backup(self) -> str:
        """返回 json/ 目录下最新的 PID 备份文件，没有则返回空字符串"""
        if not os.path.isdir(BACKUP_DIR):
            return ""

        backups = [
            os.path.join(BACKUP_DIR, name)
            for name in os.listdir(BACKUP_DIR)
            if name.startswith('servo_pid_backup_') and name.endswith('.json')
        ]
        if not backups:
            return ""
        return max(backups, key=os.path.getmtime)

    def start_write(self, changes_by_id: dict):
        """统一写入入口：更新状态并交给后台线程"""
        for servo_id in changes_by_id:
            self.set_status(servo_id, "写入中...")
        self.writing = True
        self.set_busy(True)
        self.write_requested.emit(changes_by_id)

    # ---------------- Worker 回调 ----------------

    @Slot(bool, str)
    def on_connection_changed(self, success: bool, port_name: str):
        self.connected = success
        if success:
            self.conn_label.setText(f"已连接: {port_name}")
            self.conn_label.setStyleSheet("color: #198754; font-weight: bold;")
            # 连接成功后自动读取
            self.servo_loaded.clear()
            for row in range(self.table.rowCount()):
                self.table.item(row, COL_STATUS).setText("读取中...")
            self.set_busy(True)
            self.read_requested.emit(list(DEFAULT_SERVO_IDS))
        else:
            self.conn_label.setText("未连接")
            self.conn_label.setStyleSheet("color: #6c757d; font-weight: bold;")
            self.servo_loaded.clear()
            for servo_id in DEFAULT_SERVO_IDS:
                for spins in (self.p_spins, self.d_spins, self.i_spins):
                    spins[servo_id].setEnabled(False)
                self.set_status(servo_id, "未连接")
            self.set_busy(False)

    @Slot(object)
    def on_scan_finished(self, found: list):
        for row, servo_id in enumerate(DEFAULT_SERVO_IDS):
            if servo_id not in found:
                self.set_status(servo_id, "未发现")
                self.p_spins[servo_id].setEnabled(False)
                self.d_spins[servo_id].setEnabled(False)
                self.i_spins[servo_id].setEnabled(False)
                item = self.table.item(row, COL_CHECK)
                item.setCheckState(Qt.CheckState.Unchecked)

    @Slot(int, object)
    def on_pid_read(self, servo_id: int, values: dict):
        if servo_id not in self.p_spins:
            return
        self.p_spins[servo_id].setValue(values['p'])
        self.d_spins[servo_id].setValue(values['d'])
        self.i_spins[servo_id].setValue(values['i'])
        for spins in (self.p_spins, self.d_spins, self.i_spins):
            spins[servo_id].setEnabled(True)
        self.servo_loaded.add(servo_id)
        if not self.writing:
            self.set_status(servo_id, "已读取", "#0d6efd")
        self.update_ui_state()

    @Slot()
    def on_read_finished(self):
        for row, servo_id in enumerate(DEFAULT_SERVO_IDS):
            item = self.table.item(row, COL_STATUS)
            if item.text() == "读取中...":
                self.set_status(servo_id, "读取失败", "#dc3545")
        self.set_busy(False)

    @Slot(int, bool, str)
    def on_write_result(self, servo_id: int, success: bool, message: str):
        self.set_status(servo_id, "写入成功" if success else "写入失败",
                        "#198754" if success else "#dc3545")

    @Slot(bool, str)
    def on_write_finished(self, all_success: bool, backup_path: str):
        self.writing = False
        self.set_busy(False)
        if all_success:
            self.add_log("✅ 全部写入成功")
        else:
            self.add_log("⚠️ 部分写入失败，请查看上方日志")
        if backup_path:
            self.add_log(f"💾 备份文件: {backup_path}")

    def set_status(self, servo_id: int, text: str, color: str = "#6c757d"):
        row = DEFAULT_SERVO_IDS.index(servo_id)
        item = self.table.item(row, COL_STATUS)
        item.setText(text)
        item.setForeground(QColor(color))

    # ---------------- 退出 ----------------

    def closeEvent(self, event):
        try:
            if self.worker_thread.isRunning():
                QMetaObject.invokeMethod(
                    self.worker, "disconnect_port",
                    Qt.ConnectionType.BlockingQueuedConnection
                )
        except Exception:
            pass

        self.worker_thread.quit()
        self.worker_thread.wait(2000)
        super().closeEvent(event)


def main():
    import argparse

    parser = argparse.ArgumentParser(description="舵机 PID 参数调节 GUI")
    parser.add_argument('--port', type=str, default=None, help='启动后自动连接的串口，如 /dev/ttyACM0')
    args, qt_args = parser.parse_known_args()
    sys.argv = [sys.argv[0]] + qt_args

    app = QApplication(sys.argv)
    if setup_light_theme:
        setup_light_theme(app)

    window = ServoPidTuneWindow(args.port)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
