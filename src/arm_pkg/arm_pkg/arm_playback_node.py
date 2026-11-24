"""Node replaying a recorded arm trajectory through the driver node."""

import csv
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from arm_msg.msg import ArmJointAngles
from arm_msg.srv import RecordControl
from arm_pkg.csv_paths import resolve_csv_path


class ArmPlaybackNode(Node):
    """Load a recorded CSV file and replay it to the cmd_angles topic."""

    def __init__(self):
        """Create the command publisher and the playback control service."""
        super().__init__('arm_playback_node')
        self.declare_parameter('playback_file', 'arm_trajectory.csv')
        self.default_file = resolve_csv_path(
            self.get_parameter('playback_file').value, 'arm_trajectory.csv'
        )

        self.publisher_ = self.create_publisher(ArmJointAngles, 'cmd_angles', 10)
        self.service = self.create_service(
            RecordControl, '/cmd_playback', self._on_playback_control
        )

        self.samples = []
        self.current_file = ''
        self.index = 0
        self.start_time = None

        self.timer = self.create_timer(0.02, self._timer_callback)

        self.get_logger().info(
            'arm_playback_node 已启动，等待 /cmd_playback 服务指令'
        )
        self.get_logger().info(
            '使用示例：\n'
            '  开始回放: ros2 service call /cmd_playback '
            'arm_msg/srv/RecordControl "{action: \'start\', '
            'filename: \'nod_trajectory.csv\'}"\n'
            '  停止回放: ros2 service call /cmd_playback '
            'arm_msg/srv/RecordControl "{action: \'stop\', filename: \'\'}"\n'
            '  查询状态: ros2 service call /cmd_playback '
            'arm_msg/srv/RecordControl "{action: \'status\', filename: \'\'}"'
        )

    def _on_playback_control(self, request, response):
        """Handle start/stop/status commands."""
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
            return False, f'正在回放: {self.current_file}，请先 stop'

        path = resolve_csv_path(filename, 'arm_trajectory.csv')
        try:
            samples = self._load_csv(path)
        except (OSError, ValueError) as exc:
            return False, f'读取文件失败 {path}: {exc}'

        if not samples:
            return False, f'文件中没有有效的轨迹数据: {path}'

        self.samples = samples
        self.current_file = path
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

                if samples:
                    if timestamp < samples[-1][0]:
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
                f'回放完成: {self.current_file}，共 {len(self.samples)} 帧'
            )
            self.start_time = None
            self.samples = []
            self.index = 0

    def _stop_playback(self):
        """Stop the current playback."""
        if self.start_time is None:
            return False, '当前没有在回放'

        message = (
            f'回放已停止: {self.current_file}，'
            f'已播放 {self.index}/{len(self.samples)} 帧'
        )
        self.get_logger().info(message)
        self.start_time = None
        self.samples = []
        self.index = 0
        return True, message

    def _get_status(self):
        """Report the current playback status."""
        if self.start_time is None:
            return True, '当前未回放'
        return (
            True,
            f'正在回放: {self.current_file}，'
            f'进度 {self.index}/{len(self.samples)} 帧',
        )


def main(args=None):
    """Initialize ROS 2 and run the playback node."""
    rclpy.init(args=args)
    node = ArmPlaybackNode()
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
