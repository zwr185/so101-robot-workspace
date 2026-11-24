"""Node replaying a shake-head trajectory when /robot_shake_head is triggered.

本节点只负责回放：摇头轨迹需提前用 arm_pkg 的录制节点录制并保存到
/home/zwr185/dev_ws/src/arm_pkg/csv 目录，再通过 shake_file 参数选择要回放的文件。

节点同时订阅舵机驱动发布的 arm_joint_angles（当前关节角度），
收到 /robot_shake_head 触发后先从当前姿态平滑过渡到轨迹首帧，再按时间轴回放，
避免机械臂从当前姿态直接跳到录制起点。
"""

import csv
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from arm_msg.msg import ArmJointAngles

# 回放用的 CSV 文件统一存放在 arm_pkg 的 csv 目录
CSV_DIR = '/home/zwr185/dev_ws/src/arm_pkg/csv'


class RobotShakeHeadNode(Node):
    """Replay a shake-head trajectory on cmd_angles when triggered."""

    def __init__(self):
        """Resolve the shake file path and create publisher, subscription, timer."""
        super().__init__('robot_shake_head_node')
        self.declare_parameter('shake_file', 'shakehead_trajectory.csv')
        self.declare_parameter('transition_duration', 1.5)
        self.shake_file = self._resolve_csv(
            self.get_parameter('shake_file').value
        )

        self.publisher_ = self.create_publisher(
            ArmJointAngles, 'cmd_angles', 10
        )
        # 回放完成信号：供 MCP 工具等待本次摇头结束
        self.done_publisher = self.create_publisher(
            Bool, '/robot_shake_head_done', 10
        )
        self.subscription = self.create_subscription(
            Bool, '/robot_shake_head', self._on_trigger, 10
        )
        # 订阅舵机驱动发布的当前关节角度，用于平滑过渡到轨迹起点
        self.angles_subscription = self.create_subscription(
            ArmJointAngles, 'arm_joint_angles', self._on_current_angles, 10
        )
        self.timer = self.create_timer(0.02, self._timer_callback)

        self.current_angles = None
        self.samples = []
        self.index = 0
        self.start_time = None

        if os.path.isfile(self.shake_file):
            self.get_logger().info(f'摇头轨迹文件: {self.shake_file}')
        else:
            self.get_logger().warn(
                f'未找到摇头轨迹文件: {self.shake_file}，'
                '请先录制并通过 shake_file 参数指定'
            )
        self.get_logger().info(
            'robot_shake_head_node 已启动，等待 /robot_shake_head 触发；'
            '手动触发示例: ros2 topic pub --once /robot_shake_head '
            'std_msgs/msg/Bool "{data: true}"'
        )

    @staticmethod
    def _resolve_csv(filename):
        """Resolve shake_file: bare names live in CSV_DIR, paths are used as-is."""
        name = (filename or '').strip() or 'shakehead_trajectory.csv'
        path = os.path.expanduser(name)
        if os.path.dirname(path):
            return path
        return os.path.join(CSV_DIR, path)

    def _on_current_angles(self, msg):
        """Keep the latest measured joint angles from the driver node."""
        if len(msg.angles) == 6:
            self.current_angles = [float(angle) for angle in msg.angles]

    def _on_trigger(self, msg):
        """Start one shake-head playback when a true message arrives."""
        if not msg.data:
            return
        if self.start_time is not None:
            self.get_logger().warn('正在摇头，忽略本次触发')
            return

        try:
            samples = self._load_csv(self.shake_file)
        except (OSError, ValueError) as exc:
            self.get_logger().error(
                f'读取摇头轨迹失败 {self.shake_file}: {exc}'
            )
            return

        if not samples:
            self.get_logger().error(
                f'摇头轨迹文件没有有效数据: {self.shake_file}'
            )
            return

        self.samples = self._with_transition(samples)
        self.index = 0
        self.start_time = time.monotonic()
        self.get_logger().info(
            f'开始摇头: {self.shake_file}，共 {len(self.samples)} 帧，'
            f'总时长 {self.samples[-1][0]:.2f} 秒'
        )

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

    def _with_transition(self, samples):
        """Prepend a smooth move from the current pose to the first sample."""
        duration = float(
            self.get_parameter('transition_duration').value
        )
        if self.current_angles is None:
            self.get_logger().warn(
                '未收到 arm_joint_angles，直接从轨迹首帧开始回放'
            )
            return samples
        if duration <= 0.0:
            return samples

        target = samples[0][1]
        steps = max(1, int(round(duration / 0.02)))
        dt = duration / steps
        transition = [
            (
                (i + 1) * dt,
                [
                    cur + (tgt - cur) * ((i + 1) / steps)
                    for cur, tgt in zip(self.current_angles, target)
                ],
            )
            for i in range(steps)
        ]

        self.get_logger().info(
            f'从当前姿态用 {duration:.2f} 秒平滑过渡到轨迹起点'
        )
        offset = transition[-1][0]
        return transition + [
            (offset + sample_time, angles)
            for sample_time, angles in samples
        ]

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
                f'摇头完成: {self.shake_file}，共 {len(self.samples)} 帧'
            )
            self.done_publisher.publish(Bool(data=True))
            self.start_time = None
            self.samples = []
            self.index = 0


def main(args=None):
    """Initialize ROS 2 and run the shake-head node."""
    rclpy.init(args=args)
    node = RobotShakeHeadNode()
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
