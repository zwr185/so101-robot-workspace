"""Follower arm driver for leader-follower teleoperation.

订阅领导臂关节角度（默认 /leader_joint_angles，通常 20ms 一帧）。
首帧固定用 1.2s 余弦缓动对齐到领导臂姿态，之后直接下发，
不再做任何平滑，保证实时跟手。

与视觉抓取链路完全解耦：
  - 输入话题默认 /leader_joint_angles
  - 反馈话题默认关闭，避免和视觉抓取的 /arm_joint_angles 冲突
  - 串口、校准文件、话题都通过 ROS 参数指定

用法 / Usage:
  ros2 run remote_control_pkg follower_arm_joint --ros-args \
    -p port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7B015685-if00 \
    -p input_topic:=/leader_joint_angles \
    -p calibration_file:=/home/zwr185/dev_ws/Seeed_RoboController/json/\
my_awesome_follower_arm.json
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from arm_msg.msg import ArmJointAngles

from remote_control_pkg.arm_calibration import (
    load_calibration,
    servo_calibrations,
)
from remote_control_pkg.servo_controller import ServoController

DEFAULT_PORT = (
    '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7B015685-if00'
)
DEFAULT_CALIBRATION_FILE = (
    '/home/zwr185/dev_ws/Seeed_RoboController/json/'
    'my_awesome_follower_arm.json'
)

ALIGN_TICK = 0.03


class FollowerArmJointNode(Node):
    """Drive the follower arm directly from 50Hz leader angle messages."""

    def __init__(self):
        """Open the follower serial port and subscribe to the leader topic."""
        super().__init__('follower_arm_joint')

        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('input_topic', '/leader_joint_angles')
        self.declare_parameter('feedback_topic', '')
        self.declare_parameter('feedback_rate', 5.0)
        self.declare_parameter('calibration_file', DEFAULT_CALIBRATION_FILE)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('align_on_start', True)
        self.declare_parameter('align_duration', 1.2)

        port = self.get_parameter('port').value
        baudrate = int(self.get_parameter('baudrate').value)
        input_topic = self.get_parameter('input_topic').value
        feedback_topic = self.get_parameter('feedback_topic').value
        feedback_rate = float(self.get_parameter('feedback_rate').value)
        calibration_file = self.get_parameter('calibration_file').value
        self.command_timeout = float(
            self.get_parameter('command_timeout').value
        )
        self.align_on_start = bool(
            self.get_parameter('align_on_start').value
        )
        self.align_duration = float(
            self.get_parameter('align_duration').value
        )

        entries = load_calibration(calibration_file)
        self.servo_ids = list(range(1, 7))
        self.controller = ServoController(
            port=port,
            baudrate=baudrate,
            calibration=servo_calibrations(entries),
        )

        if not self.controller.is_open:
            self.get_logger().warn(
                f'无法打开从动臂串口 {port}，节点将以空转模式运行'
            )
        if not entries and calibration_file:
            self.get_logger().warn(
                f'校准文件无效，沿用默认映射: {calibration_file}'
            )

        self._torque_enabled = False
        self._aligned = False
        self._align_active = False
        self._align_start = {}
        self._align_end = {}
        self._align_t0 = 0.0
        self._last_cmd_time = 0.0
        self._link_warned = False

        # 启动时先释放力矩，方便把从动臂摆到与领导臂接近的姿态
        for servo_id in self.servo_ids:
            self.controller.disable_torque(servo_id)
            time.sleep(0.05)
        self.get_logger().info(
            '已释放从动臂力矩，请先将两套机械臂摆到接近的姿态'
        )

        self.align_timer = self.create_timer(ALIGN_TICK, self._align_tick)
        self.watchdog_timer = self.create_timer(
            1.0, self._check_command_timeout
        )
        self.subscription = self.create_subscription(
            ArmJointAngles, input_topic, self._on_leader_angles, 10
        )

        self.feedback_publisher = None
        if feedback_topic and feedback_rate > 0.0:
            self.feedback_publisher = self.create_publisher(
                ArmJointAngles, feedback_topic, 10
            )
            self.feedback_timer = self.create_timer(
                1.0 / feedback_rate, self._publish_feedback
            )

        self.get_logger().info(
            f'follower_arm_joint 已启动，串口: {port}，'
            f'订阅领导臂: {input_topic}'
        )
        if self.feedback_publisher is not None:
            self.get_logger().info(
                f'反馈话题: {feedback_topic}（{feedback_rate:.1f}Hz）'
            )

    # --- 领导臂指令 ---

    def _on_leader_angles(self, msg):
        """Apply the latest leader angles directly to the follower servos."""
        values = [float(angle) for angle in msg.angles]
        if len(values) != 6:
            self.get_logger().warn(
                f'收到无效的领导臂角度数量: {len(values)}，期望 6 个'
            )
            return
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warn('收到非有限数值的角度，已忽略')
            return

        now = time.monotonic()
        if self._link_warned:
            self.get_logger().info('领导臂指令已恢复')
            self._link_warned = False
        self._last_cmd_time = now

        targets = {
            servo_id: self.controller.angle_to_position(servo_id, angle)
            for servo_id, angle in zip(self.servo_ids, values)
        }

        self._enable_all_torque()

        # 启动对齐阶段：只更新目标终点，由定时器插值下发
        if self._align_active:
            self._align_end = targets
            return

        if not self._aligned:
            if self.align_on_start and self._start_alignment(targets):
                return
            self._aligned = True

        self._write_targets(targets)

    def _enable_all_torque(self):
        """Enable torque on all servos once."""
        if self._torque_enabled:
            return
        for servo_id in self.servo_ids:
            self.controller.enable_torque(servo_id)
        self._torque_enabled = True

    def _write_targets(self, targets):
        """Write all raw target positions immediately."""
        for servo_id in self.servo_ids:
            self.controller.write_position(servo_id, targets[servo_id])

    # --- 启动对齐 ---

    def _start_alignment(self, targets):
        """Start the fixed startup alignment before direct tracking."""
        if self.align_duration <= 0.0:
            return False

        current = {}
        for servo_id in self.servo_ids:
            position = self.controller.read_raw_position(servo_id)
            if position is None:
                self.get_logger().warn(
                    '读取从动臂当前位置失败，直接进入实时跟随'
                )
                return False
            current[servo_id] = position

        self._align_start = current
        self._align_end = dict(targets)
        self._align_t0 = time.monotonic()
        self._align_active = True
        self.get_logger().info(
            f'首帧用 {self.align_duration:.1f}s 平滑对齐到领导臂姿态，'
            '之后进入实时直写跟随'
        )
        return True

    def _align_tick(self):
        """Cosine-interpolate from the start pose to the latest target."""
        if not self._align_active:
            return

        elapsed = time.monotonic() - self._align_t0
        t = min(elapsed / self.align_duration, 1.0)
        s = 0.5 - 0.5 * math.cos(math.pi * t)

        for servo_id in self.servo_ids:
            start = self._align_start[servo_id]
            end = self._align_end[servo_id]
            position = int(round(start + (end - start) * s))
            self.controller.write_position(servo_id, position)

        if t >= 1.0:
            self._align_active = False
            self._aligned = True
            self.get_logger().info(
                '启动对齐完成，进入 20ms 实时跟随'
            )

    # --- 监控与反馈 ---

    def _check_command_timeout(self):
        """Warn once when the leader stream stops."""
        if self._last_cmd_time == 0.0 or self._link_warned:
            return
        if self.command_timeout <= 0.0:
            return
        if time.monotonic() - self._last_cmd_time > self.command_timeout:
            self.get_logger().warn(
                f'超过 {self.command_timeout:.2f}s 未收到领导臂指令，'
                '从动臂保持当前位置'
            )
            self._link_warned = True

    def _publish_feedback(self):
        """Publish the measured follower angles (optional)."""
        if self.feedback_publisher is None:
            return

        angles = []
        for servo_id in self.servo_ids:
            angle = self.controller.read_angle(servo_id)
            if angle is None:
                return
            angles.append(float(angle))

        msg = ArmJointAngles()
        msg.angles = angles
        self.feedback_publisher.publish(msg)

    def destroy_node(self):
        """Close the serial port before destroying the node."""
        if hasattr(self, 'controller') and self.controller:
            self.controller.close()
        super().destroy_node()


def main(args=None):
    """Initialize ROS 2 and run the follower teleoperation driver."""
    rclpy.init(args=args)
    node = FollowerArmJointNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
