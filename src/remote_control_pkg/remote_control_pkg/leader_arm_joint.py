"""Leader arm reader for leader-follower teleoperation.

启动时释放领导臂所有舵机力矩，让操作者可以自由拖动；
然后以固定频率（默认 50Hz，即 20ms 一帧）读取 6 个关节角度，
发布到 /leader_joint_angles，供 follower_arm_joint 直接跟随。

用法 / Usage:
  ros2 run remote_control_pkg leader_arm_joint --ros-args \
    -p port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7B016147-if00 \
    -p output_topic:=/leader_joint_angles \
    -p calibration_file:=/home/zwr185/dev_ws/Seeed_RoboController/json/\
my_awesome_leader_arm.json
"""

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
    '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7B016147-if00'
)
DEFAULT_CALIBRATION_FILE = (
    '/home/zwr185/dev_ws/Seeed_RoboController/json/'
    'my_awesome_leader_arm.json'
)


class LeaderArmJointNode(Node):
    """Read leader joint angles and publish them for the follower driver."""

    def __init__(self):
        """Open the leader serial port, release torque and create the publisher."""
        super().__init__('leader_arm_joint')

        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('output_topic', '/leader_joint_angles')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('calibration_file', DEFAULT_CALIBRATION_FILE)
        self.declare_parameter('release_torque', True)

        port = self.get_parameter('port').value
        baudrate = int(self.get_parameter('baudrate').value)
        output_topic = self.get_parameter('output_topic').value
        publish_rate = max(
            float(self.get_parameter('publish_rate').value), 1.0
        )
        calibration_file = self.get_parameter('calibration_file').value
        release_torque = bool(
            self.get_parameter('release_torque').value
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
                f'无法打开领导臂串口 {port}，节点将以空转模式运行'
            )
        if not entries and calibration_file:
            self.get_logger().warn(
                f'校准文件无效，沿用默认映射: {calibration_file}'
            )

        if release_torque:
            for servo_id in self.servo_ids:
                self.controller.disable_torque(servo_id)
                time.sleep(0.05)
            self.get_logger().info(
                '已释放领导臂力矩，可以手动拖动领导臂'
            )

        self.publisher = self.create_publisher(
            ArmJointAngles, output_topic, 10
        )
        self.read_failures = 0
        self.timer = self.create_timer(
            1.0 / publish_rate, self._publish_angles
        )

        self.get_logger().info(
            f'leader_arm_joint 已启动，串口: {port}，'
            f'发布: {output_topic}（{publish_rate:.1f}Hz）'
        )

    def _publish_angles(self):
        """Read all leader servos and publish one angle frame."""
        angles = []
        for servo_id in self.servo_ids:
            angle = self.controller.read_angle(servo_id)
            if angle is None:
                self.read_failures += 1
                if self.read_failures == 1 or self.read_failures % 100 == 0:
                    self.get_logger().warn(
                        f'读取领导臂 ID{servo_id} 角度失败'
                        f'（连续 {self.read_failures} 次），跳过本帧'
                    )
                return
            angles.append(float(angle))

        if self.read_failures:
            self.get_logger().info('领导臂角度读取已恢复')
            self.read_failures = 0

        msg = ArmJointAngles()
        msg.angles = angles
        self.publisher.publish(msg)

    def destroy_node(self):
        """Close the serial port before destroying the node."""
        if hasattr(self, 'controller') and self.controller:
            self.controller.close()
        super().destroy_node()


def main(args=None):
    """Initialize ROS 2 and run the leader teleoperation reader."""
    rclpy.init(args=args)
    node = LeaderArmJointNode()
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
