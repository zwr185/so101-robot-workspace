"""GUI node for recording and playing back arm trajectories."""

import csv
import os
import sys
import time

from arm_msg.msg import ArmJointAngles
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
import rclpy
from rclpy.node import Node

from arm_pkg.csv_paths import resolve_csv_path


class GuiRecordPlaybackNode(Node):
    """Record arm angles and replay recordings, never at the same time."""

    def __init__(self):
        """Create the subscription, publisher and playback timer."""
        super().__init__('gui_record_playback_node')
        self.declare_parameter('record_file', 'arm_trajectory.csv')
        self.default_file = resolve_csv_path(
            self.get_parameter('record_file').value, 'arm_trajectory.csv'
        )

        # Recording state
        self.file = None
        self.writer = None
        self.record_file = ''
        self.sample_count = 0

        # Playback state
        self.samples = []
        self.playback_file = ''
        self.index = 0
        self.start_time = None

        self.subscription = self.create_subscription(
            ArmJointAngles, 'arm_joint_angles', self._on_angles, 10
        )
        self.publisher_ = self.create_publisher(
            ArmJointAngles, 'cmd_angles', 10
        )
        self.timer = self.create_timer(0.02, self._timer_callback)

        self.get_logger().info('gui_record_playback_node 已启动')

    def is_recording(self):
        """Return True when recording is active."""
        return self.file is not None

    def is_playing(self):
        """Return True when playback is active."""
        return self.start_time is not None

    def recording_progress(self):
        """Return the number of recorded samples."""
        return self.sample_count

    def playback_progress(self):
        """Return (played, total) frames of the current playback."""
        return self.index, len(self.samples)

    # --- recording ---

    def _on_angles(self, msg):
        """Append one timestamped sample when recording is active."""
        if self.file is None:
            return

        timestamp = self.get_clock().now().nanoseconds / 1e9
        row = [f'{timestamp:.9f}']
        row.extend(f'{float(angle):.3f}' for angle in msg.angles)

        self.writer.writerow(row)
        self.file.flush()
        self.sample_count += 1
        if self.sample_count % 100 == 0:
            self.get_logger().info(f'已录制 {self.sample_count} 条数据')

    def start_recording(self, filename):
        """Open the target file and start recording."""
        if self.file is not None:
            return False, f'正在录制中: {self.record_file}，请先停止'
        if self.start_time is not None:
            return (
                False,
                f'正在回放: {self.playback_file}，不能同时录制，请先停止回放',
            )

        path = resolve_csv_path(filename, 'arm_trajectory.csv')
        directory = os.path.dirname(path)
        if directory and not os.path.isdir(directory):
            return False, f'目录不存在: {directory}'

        try:
            self.file = open(path, 'w', newline='', encoding='utf-8')
        except OSError as exc:
            self.file = None
            return False, f'无法创建文件 {path}: {exc}'

        self.writer = csv.writer(self.file)
        self.writer.writerow(
            ['timestamp_sec'] + [f'joint_{i}' for i in range(1, 7)]
        )
        self.file.flush()
        self.record_file = path
        self.sample_count = 0

        self.get_logger().info(f'开始录制，保存到: {path}')
        return True, f'已开始录制: {path}'

    def stop_recording(self):
        """Close the file and stop recording."""
        if self.file is None:
            return False, '当前没有在录制'

        self.file.close()
        self.file = None
        self.writer = None

        message = (
            f'录制已停止，文件: {self.record_file}，'
            f'共 {self.sample_count} 条数据'
        )
        self.get_logger().info(message)
        return True, message

    # --- playback ---

    def start_playback(self, filename):
        """Load the CSV file and start replaying it."""
        if self.start_time is not None:
            return False, f'正在回放: {self.playback_file}，请先停止'
        if self.file is not None:
            return (
                False,
                f'正在录制: {self.record_file}，不能同时回放，请先停止录制',
            )

        path = resolve_csv_path(filename, 'arm_trajectory.csv')
        try:
            samples = self._load_csv(path)
        except (OSError, ValueError) as exc:
            return False, f'读取文件失败 {path}: {exc}'

        if not samples:
            return False, f'文件中没有有效的轨迹数据: {path}'

        self.samples = samples
        self.playback_file = path
        self.index = 0
        self.start_time = time.monotonic()

        self.get_logger().info(
            f'开始回放: {path}，共 {len(samples)} 帧，'
            f'时长 {samples[-1][0]:.2f} 秒'
        )
        return True, f'已开始回放: {path}，共 {len(samples)} 帧'

    @staticmethod
    def _load_csv(path):
        """Parse the recorded CSV into (relative_time, angles) samples."""
        samples = []
        with open(path, 'r', newline='', encoding='utf-8') as csv_file:
            reader = csv.reader(csv_file)
            next(reader, None)  # skip header
            for line_number, row in enumerate(reader, start=2):
                if not row:
                    continue
                if len(row) < 7:
                    raise ValueError(f'第 {line_number} 行列数不足')
                try:
                    timestamp = float(row[0])
                    angles = [float(value) for value in row[1:7]]
                except ValueError:
                    raise ValueError(f'第 {line_number} 行数据格式错误')

                if samples and timestamp < samples[-1][0]:
                    raise ValueError(f'第 {line_number} 行时间戳倒序')

                samples.append((timestamp, angles))

        if not samples:
            return samples

        base = samples[0][0]
        return [(timestamp - base, angles) for timestamp, angles in samples]

    def _timer_callback(self):
        """Publish every sample whose playback time has come."""
        if self.start_time is None:
            return

        elapsed = time.monotonic() - self.start_time
        while self.index < len(self.samples):
            sample_time, angles = self.samples[self.index]
            if sample_time > elapsed:
                break

            msg = ArmJointAngles()
            msg.angles = [float(angle) for angle in angles]
            self.publisher_.publish(msg)
            self.index += 1

        if self.index >= len(self.samples):
            self.get_logger().info(
                f'回放完成: {self.playback_file}，共 {len(self.samples)} 帧'
            )
            self.start_time = None
            self.samples = []
            self.index = 0

    def stop_playback(self):
        """Stop the current playback."""
        if self.start_time is None:
            return False, '当前没有在回放'

        message = (
            f'回放已停止: {self.playback_file}，'
            f'已播放 {self.index}/{len(self.samples)} 帧'
        )
        self.get_logger().info(message)
        self.start_time = None
        self.samples = []
        self.index = 0
        return True, message

    # --- shared ---

    def reset_to_zero(self):
        """Publish a command moving every joint to 0 degrees."""
        if self.file is not None:
            return False, '正在录制，请先停止录制再复位'
        if self.start_time is not None:
            return False, '正在回放，请先停止回放再复位'

        msg = ArmJointAngles()
        msg.angles = [0.0] * 6
        self.publisher_.publish(msg)
        self.get_logger().info('已发送复位指令：全部关节角度为 0 度')
        return True, '已发送复位指令（全部关节 0 度）'

    def get_status(self):
        """Return a short recording and playback status text."""
        if self.file is not None:
            return (
                f'正在录制：{self.record_file}'
                f'（已记录 {self.sample_count} 条）'
            )
        if self.start_time is not None:
            played, total = self.playback_progress()
            return f'正在回放：{self.playback_file}（进度 {played}/{total} 帧）'
        return '空闲'

    def shutdown(self):
        """Close the CSV file before shutting down."""
        if self.file is not None:
            self.file.close()
            self.file = None
            self.get_logger().info(
                f'录制结束，文件: {self.record_file}，'
                f'共 {self.sample_count} 条数据'
            )


class RecordPlaybackWindow(QWidget):
    """Graphical interface for recording and playback control."""

    def __init__(self, node):
        """Store the ROS node, build the UI and start status refresh."""
        super().__init__()
        self.node = node
        self._init_ui()

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._refresh)
        self.status_timer.start(200)

    def _init_ui(self):
        self.setWindowTitle('机械臂录制回放控制器')
        self.resize(620, 380)
        self.setStyleSheet(
            """
            QWidget {
                background-color: #f7f7f7;
                font-size: 16px;
            }
            QLabel {
                color: #222222;
            }
            QLineEdit {
                background-color: #ffffff;
                border: 1px solid #b5b5b5;
                border-radius: 6px;
                padding: 6px 8px;
            }
            QPushButton {
                background-color: #ffffff;
                border: 1px solid #b5b5b5;
                border-radius: 6px;
                min-height: 40px;
            }
            QPushButton:hover {
                background-color: #ededed;
            }
            QPushButton:pressed {
                background-color: #dcdcdc;
            }
            QPushButton:disabled {
                color: #a0a0a0;
                background-color: #eeeeee;
            }
            """
        )

        self.file_edit = QLineEdit(self.node.default_file)

        self.start_record_button = QPushButton('开始录制')
        self.stop_record_button = QPushButton('停止录制')
        self.start_playback_button = QPushButton('开始回放')
        self.stop_playback_button = QPushButton('停止回放')
        self.reset_button = QPushButton('复位（全部 0 度）')

        self.start_record_button.clicked.connect(self._on_start_record)
        self.stop_record_button.clicked.connect(self._on_stop_record)
        self.start_playback_button.clicked.connect(self._on_start_playback)
        self.stop_playback_button.clicked.connect(self._on_stop_playback)
        self.reset_button.clicked.connect(self._on_reset)

        self.status_label = QLabel('空闲')
        self.status_label.setWordWrap(True)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        form = QGridLayout()
        form.setHorizontalSpacing(12)
        form.setVerticalSpacing(12)
        form.addWidget(QLabel('文件名'), 0, 0)
        form.addWidget(self.file_edit, 0, 1)
        form.setColumnStretch(1, 1)

        record_row = QHBoxLayout()
        record_row.setSpacing(12)
        record_row.addWidget(QLabel('录制'))
        record_row.addWidget(self.start_record_button)
        record_row.addWidget(self.stop_record_button)

        playback_row = QHBoxLayout()
        playback_row.setSpacing(12)
        playback_row.addWidget(QLabel('回放'))
        playback_row.addWidget(self.start_playback_button)
        playback_row.addWidget(self.stop_playback_button)

        reset_row = QHBoxLayout()
        reset_row.setSpacing(12)
        reset_row.addWidget(QLabel('复位'))
        reset_row.addWidget(self.reset_button)

        layout.addLayout(form)
        layout.addLayout(record_row)
        layout.addLayout(playback_row)
        layout.addLayout(reset_row)
        layout.addWidget(QLabel('当前状态'))
        layout.addWidget(self.status_label)
        layout.addStretch(1)

        self._refresh()

    def _on_start_record(self):
        ok, message = self.node.start_recording(self.file_edit.text().strip())
        if not ok:
            QMessageBox.warning(self, '无法开始录制', message)
        self._refresh()

    def _on_stop_record(self):
        ok, message = self.node.stop_recording()
        if not ok:
            QMessageBox.information(self, '提示', message)
        else:
            QMessageBox.information(self, '录制结束', message)
        self._refresh()

    def _on_start_playback(self):
        ok, message = self.node.start_playback(self.file_edit.text().strip())
        if not ok:
            QMessageBox.warning(self, '无法开始回放', message)
        self._refresh()

    def _on_stop_playback(self):
        ok, message = self.node.stop_playback()
        if not ok:
            QMessageBox.information(self, '提示', message)
        else:
            QMessageBox.information(self, '回放结束', message)
        self._refresh()

    def _on_reset(self):
        ok, message = self.node.reset_to_zero()
        if ok:
            QMessageBox.information(self, '复位', message)
        else:
            QMessageBox.warning(self, '无法复位', message)
        self._refresh()

    def _refresh(self):
        recording = self.node.is_recording()
        playing = self.node.is_playing()

        self.status_label.setText(self.node.get_status())
        self.start_record_button.setEnabled(not recording and not playing)
        self.stop_record_button.setEnabled(recording)
        self.start_playback_button.setEnabled(not recording and not playing)
        self.stop_playback_button.setEnabled(playing)
        self.reset_button.setEnabled(not recording and not playing)

    def closeEvent(self, event):
        """Stop recording and playback before closing the window."""
        if self.node.is_recording():
            self.node.stop_recording()
        if self.node.is_playing():
            self.node.stop_playback()
        super().closeEvent(event)


def main(args=None):
    """Initialize ROS 2 and the PyQt5 interface, then run the node."""
    rclpy.init(args=args)
    node = GuiRecordPlaybackNode()

    app = QApplication(sys.argv)
    window = RecordPlaybackWindow(node)
    window.show()

    timer = QTimer()

    def spin_ros():
        if not rclpy.ok():
            timer.stop()
            app.quit()
            return
        rclpy.spin_once(node, timeout_sec=0)

    timer.timeout.connect(spin_ros)
    timer.start(50)

    exit_code = 0
    try:
        exit_code = app.exec_()
    finally:
        timer.stop()
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
