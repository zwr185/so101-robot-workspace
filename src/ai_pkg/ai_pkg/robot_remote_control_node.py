"""Voice-triggered teleoperation controller.

订阅 /robot_remote_control（Bool: True=开启，False=停止），
向 /teleop_enable 发布遥操作使能状态，由 arm_playback_joint_node 消费。
同时监控 /leader_joint_angles，领导臂角度中断时自动关闭遥操作。
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from arm_msg.msg import ArmJointAngles


class RobotRemoteControlNode(Node):
    """Enable or disable leader-follower teleoperation on voice command."""

    def __init__(self):
        """Create the trigger subscription, enable publisher and watchdog."""
        super().__init__('robot_remote_control_node')
        self.declare_parameter('trigger_topic', '/robot_remote_control')
        self.declare_parameter('enable_topic', '/teleop_enable')
        self.declare_parameter('leader_topic', '/leader_joint_angles')
        self.declare_parameter('leader_timeout', 1.0)

        trigger_topic = self.get_parameter('trigger_topic').value
        enable_topic = self.get_parameter('enable_topic').value
        leader_topic = self.get_parameter('leader_topic').value
        self.leader_timeout = float(
            self.get_parameter('leader_timeout').value
        )

        self.enabled = False
        self.last_leader_time = 0.0
        self.timeout_warned = False

        self.enable_publisher = self.create_publisher(
            Bool, enable_topic, 10
        )
        self.create_subscription(Bool, trigger_topic, self._on_trigger, 10)
        self.create_subscription(
            ArmJointAngles, leader_topic, self._on_leader_angles, 10
        )
        self.timer = self.create_timer(1.0, self._check_leader_timeout)

        # 启动时确保遥操作处于关闭状态
        self.enable_publisher.publish(Bool(data=False))
        self.get_logger().info(
            f'robot_remote_control_node 已启动，触发: {trigger_topic}，'
            f'使能输出: {enable_topic}'
        )

    def _on_trigger(self, msg):
        """Start or stop teleoperation from a Bool trigger."""
        enable = bool(msg.data)
        if enable == self.enabled:
            return

        self.enabled = enable
        self.timeout_warned = False
        self.enable_publisher.publish(Bool(data=enable))

        if enable:
            self.get_logger().info('遥操作已开启')
            if self.last_leader_time == 0.0:
                self.get_logger().warn(
                    '尚未收到领导臂角度，请确认 leader_arm_joint 已启动'
                )
        else:
            self.get_logger().info('遥操作已停止')

    def _on_leader_angles(self, msg):
        """Track the latest leader frame for the timeout watchdog."""
        if len(msg.angles) == 6:
            self.last_leader_time = time.monotonic()

    def _check_leader_timeout(self):
        """Auto-disable teleoperation if the leader stream stops."""
        if not self.enabled or self.timeout_warned:
            return
        if self.last_leader_time == 0.0:
            return
        if time.monotonic() - self.last_leader_time > self.leader_timeout:
            self.get_logger().warn('领导臂角度中断，自动停止遥操作')
            self.enabled = False
            self.timeout_warned = True
            self.enable_publisher.publish(Bool(data=False))

    def destroy_node(self):
        """Make sure teleoperation is disabled on shutdown."""
        if self.enabled:
            self.enable_publisher.publish(Bool(data=False))
        super().destroy_node()


def main(args=None):
    """Initialize ROS 2 and run the teleoperation controller."""
    rclpy.init(args=args)
    node = RobotRemoteControlNode()
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
