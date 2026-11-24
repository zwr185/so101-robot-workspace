"""Node recording the arm joint trajectory to a CSV file on demand."""

import csv
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from arm_msg.msg import ArmJointAngles
from arm_msg.srv import RecordControl
from arm_pkg.csv_paths import CSV_DIR, resolve_csv_path


class ArmRecordNode(Node):
    """Record timestamped arm joint samples, controlled by a service."""

    def __init__(self):
        """Create the subscription and the record control service."""
        super().__init__('arm_record_node')
        self.declare_parameter('output_file', 'arm_trajectory.csv')
        self.default_file = resolve_csv_path(
            self.get_parameter('output_file').value, 'arm_trajectory.csv'
        )

        self.file = None
        self.writer = None
        self.current_file = ''
        self.sample_count = 0

        self.subscription = self.create_subscription(
            ArmJointAngles, 'arm_joint_angles', self._on_angles, 10
        )
        self.service = self.create_service(
            RecordControl, '/cmd_record', self._on_record_control
        )
        self.get_logger().info('arm_record_node 已启动，等待 /cmd_record 服务指令')
        self.get_logger().info(
            f'录制文件保存在: {CSV_DIR}\n'
            '使用示例：\n'
            '  开始录制(指定文件): ros2 service call /cmd_record '
            'arm_msg/srv/RecordControl "{action: \'start\', '
            'filename: \'arm_01.csv\'}"\n'
            '  开始录制(默认文件): ros2 service call /cmd_record '
            'arm_msg/srv/RecordControl "{action: \'start\', filename: \'\'}"\n'
            '  停止录制: ros2 service call /cmd_record '
            'arm_msg/srv/RecordControl "{action: \'stop\', filename: \'\'}"\n'
            '  查询状态: ros2 service call /cmd_record '
            'arm_msg/srv/RecordControl "{action: \'status\', filename: \'\'}"'
        )

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
        """Handle start/stop/status commands."""
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
            return (
                False,
                f'正在录制中: {self.current_file}，请先 stop',
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
        self.current_file = path
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

        message = f'录制已停止，文件: {self.current_file}，共 {self.sample_count} 条数据'
        self.get_logger().info(message)
        return True, message

    def _get_status(self):
        """Report the current recording status."""
        if self.file is None:
            return True, '当前未录制'
        return (
            True,
            f'正在录制: {self.current_file}，已记录 {self.sample_count} 条数据',
        )

    def destroy_node(self):
        """Close the CSV file before destroying the node."""
        if self.file is not None:
            self.file.close()
            self.file = None
            print(
                f'录制结束，文件: {self.current_file}，共 {self.sample_count} 条数据',
                flush=True,
            )
        super().destroy_node()


def main(args=None):
    """Initialize ROS 2 and run the recording node."""
    rclpy.init(args=args)
    node = ArmRecordNode()
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
