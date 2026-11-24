"""Node driving the arm servos from the solved joint angles."""

import time

from arm_msg.msg import ArmJointAngles
from arm_pkg.arm_calibration import load_calibration, servo_calibrations
from arm_pkg.arm_joint_node import ServoController
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ArmCatchJointNode(Node):
    """Subscribe to the solved angles and drive all six servos."""

    def __init__(self):
        """Open the serial port and create the angle subscription."""
        super().__init__('arm_catch_joint_node')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('input_topic', '/arm_catch_angles')
        self.declare_parameter('feedback_topic', '/arm_current_angles')
        self.declare_parameter('feedback_rate', 5.0)
        self.declare_parameter('calibration_file', '')

        port = self.get_parameter('port').value
        input_topic = self.get_parameter('input_topic').value
        feedback_topic = self.get_parameter('feedback_topic').value
        feedback_rate = float(self.get_parameter('feedback_rate').value)
        calibration_file = self.get_parameter('calibration_file').value
        entries = load_calibration(calibration_file)
        self.servo_ids = list(range(1, 7))

        self.controller = ServoController(
            port=port,
            calibration=servo_calibrations(entries),
        )
        if not self.controller.serial_port or not self.controller.serial_port.is_open:
            self.get_logger().warn(
                f'Failed to open serial port {port}. Running in dummy mode?'
            )

        # Disable torque for all servos to allow manual movement
        for servo_id in self.servo_ids:
            self.controller.disable_torque(servo_id)
            time.sleep(0.05)

        self.subscription = self.create_subscription(
            ArmJointAngles, input_topic, self._on_angles, 10
        )
        self.feedback_publisher_ = self.create_publisher(
            ArmJointAngles, feedback_topic, 10
        )
        self.feedback_timer = self.create_timer(
            1.0 / max(feedback_rate, 1.0), self._publish_current_angles
        )
        self.get_logger().info(
            f'arm_catch_joint_node 已启动，串口: {port}，订阅: {input_topic}，'
            f'反馈: {feedback_topic}'
        )
        if entries:
            self.get_logger().info(
                f'已加载校准文件: {calibration_file}，'
                f'关节: {list(entries.keys())}'
            )
        elif calibration_file:
            self.get_logger().warn(f'校准文件无效，沿用默认映射: {calibration_file}')

    def _publish_current_angles(self):
        """Publish the measured joint angles for a smooth approach start."""
        values = [
            self.controller.get_servo_angle(servo_id)
            for servo_id in self.servo_ids
        ]
        if any(value is None for value in values):
            return

        msg = ArmJointAngles()
        msg.angles = [float(value) for value in values]
        self.feedback_publisher_.publish(msg)

    def _on_angles(self, msg):
        """Drive every servo to the received angles."""
        values = [float(angle) for angle in msg.angles]

        if len(values) != 6:
            self.get_logger().warn(
                f'收到无效的角度数量: {len(values)}，期望 6 个'
            )
            return

        for servo_id, angle in zip(self.servo_ids, values):
            self.controller.enable_torque(servo_id)
            self.controller.set_servo_angle(servo_id, angle)

    def destroy_node(self):
        """Close the serial port before destroying the node."""
        if hasattr(self, 'controller') and self.controller:
            self.controller.close()
        super().destroy_node()


def main(args=None):
    """Initialize ROS 2 and run the arm catch joint node."""
    rclpy.init(args=args)
    node = ArmCatchJointNode()
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
