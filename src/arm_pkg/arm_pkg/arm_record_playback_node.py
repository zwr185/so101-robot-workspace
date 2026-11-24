"""Node combining trajectory recording and playback for the arm."""

import csv
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from arm_msg.msg import ArmJointAngles
from arm_msg.srv import RecordControl
from arm_pkg.csv_paths import resolve_csv_path


class ArmRecordPlaybackNode(Node):
    """Record arm angles and replay recordings, never at the same time."""

    def __init__(self):
        """Create the subscription, publisher, services and timer."""
        super().__init__('arm_record_playback_node')
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
        self.publisher_ = self.create_publisher(ArmJointAngles, 'cmd_angles', 10)
        self.record_service = self.create_service(
            RecordControl, '/cmd_record', self._on_record_control
        )
        self.playback_service = self.create_service(
            RecordControl, '/cmd_playback', self._on_playback_control
        )
        self.timer = self.create_timer(0.02, self._timer_callback)

        self.get_logger().info(
            'arm_record_playback_node 已启动，等待 /cmd_record 与 '
            '/cmd_playback 服务指令'
        )
        self.get_logger().info(
            '使用示例：\n'
            '  开始录制: ros2 service call /cmd_record '
            'arm_msg/srv/RecordControl "{action: \'start\', '
            'filename: \'arm_01.csv\'}"\n'
            '  停止录制: ros2 service call /cmd_record '
            'arm_msg/srv/RecordControl "{action: \'stop\', filename: \'\'}"\n'
            '  开始回放: ros2 service call /cmd_playback '
            'arm_msg/srv/RecordControl "{action: \'start\', '
            'filename: \'arm_01.csv\'}"\n'
            '  停止回放: ros2 service call /cmd_playback '
            'arm_msg/srv/RecordControl "{action: \'stop\', filename: \'\'}"\n'
            '  查询状态: ros2 service call /cmd_record '
            'arm_msg/srv/RecordControl "{action: \'status\', filename: \'\'}"'
        )

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

    def _on_record_control(self, request, response):
        """Handle record start/stop/status commands."""
        action = request.action.strip().lower()

        if action == 'start':
            response.success, response.message = self._start_recording(
                request.filename.strip()
            )
        elif action == 'stop':
            response.success, response.message = self._stop_recording()
        elif action == 'status':
            response.success, response.message = self._get_status()
        else:
            response.success = False
            response.message = (
                f'未知指令: {request.action}，支持 start / stop / status'
            )

        return response

    def _start_recording(self, filename):
        """Open the target file and start recording."""
        if self.file is not None:
            return False, f'正在录制中: {self.record_file}，请先 stop'
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

    def _stop_recording(self):
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

    def _on_playback_control(self, request, response):
        """Handle playback start/stop/status commands."""
        action = request.action.strip().lower()

        if action == 'start':
            response.success, response.message = self._start_playback(
                request.filename.strip()
            )
        elif action == 'stop':
            response.success, response.message = self._stop_playback()
        elif action == 'status':
            response.success, response.message = self._get_status()
        else:
            response.success = False
            response.message = (
                f'未知指令: {request.action}，支持 start / stop / status'
            )

        return response

    def _start_playback(self, filename):
        """Load the CSV file and start replaying it."""
        if self.start_time is not None:
            return False, f'正在回放: {self.playback_file}，请先 stop'
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

    def _stop_playback(self):
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

    def _get_status(self):
        """Report both recording and playback status."""
        if self.file is None:
            record_status = '当前未录制'
        else:
            record_status = (
                f'正在录制: {self.record_file}，已记录 {self.sample_count} 条数据'
            )

        if self.start_time is None:
            playback_status = '当前未回放'
        else:
            playback_status = (
                f'正在回放: {self.playback_file}，'
                f'进度 {self.index}/{len(self.samples)} 帧'
            )

        return True, f'{record_status}；{playback_status}'

    def destroy_node(self):
        """Close the CSV file before destroying the node."""
        if self.file is not None:
            self.file.close()
            self.file = None
            print(
                f'录制结束，文件: {self.record_file}，共 {self.sample_count} 条数据',
                flush=True,
            )
        super().destroy_node()


def main(args=None):
    """Initialize ROS 2 and run the record/playback node."""
    rclpy.init(args=args)
    node = ArmRecordPlaybackNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
