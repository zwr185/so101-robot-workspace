"""Joint driver: direct playback + smooth visual catch + teleoperation.

用于需要同时处理多类指令的场景（语音控制、录制回放）：
  - 回放（cmd_angles）：快速直写，保持原本的回放速度；
  - 视觉抓取（/arm_catch_angles）：沿用 1.2s 平滑控制；
  - 遥操作（/leader_joint_angles）：收到 /teleop_enable=True 后跟随领导臂，
    每帧限幅直写，避免首次使能时大幅跳变。

这些指令共用同一个串口，因此本节点继承 ArmCatchJointNode，作为唯一的
串口驱动进程。遥操作开启期间会忽略回放和视觉指令，避免互相拉扯。
"""

import math

from arm_msg.msg import ArmJointAngles
from arm_pkg.arm_catch_joint_node import ArmCatchJointNode
import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Bool

TELEOP_MAX_STEP = 40  # 遥操作每帧最大原始位置变化量（约 3.5 度）


class ArmPlaybackJointNode(ArmCatchJointNode):
    """Drive servos fast for playback, smoothly for vision, and gated for teleop."""

    def __init__(self):
        """Add playback and teleoperation subscriptions on top of the visual driver."""
        super().__init__()
        self.declare_parameter('playback_topic', 'cmd_angles')
        self.declare_parameter('teleop_topic', '/leader_joint_angles')
        self.declare_parameter('teleop_enable_topic', '/teleop_enable')
        self.declare_parameter('teleop_max_step', TELEOP_MAX_STEP)

        playback_topic = self.get_parameter('playback_topic').value
        teleop_topic = self.get_parameter('teleop_topic').value
        teleop_enable_topic = self.get_parameter(
            'teleop_enable_topic'
        ).value
        self.teleop_max_step = max(
            int(self.get_parameter('teleop_max_step').value), 1
        )

        self._teleop_enabled = False
        self._teleop_positions = {}

        self.playback_subscription = self.create_subscription(
            ArmJointAngles, playback_topic, self._on_playback_angles, 10
        )
        self.teleop_enable_subscription = self.create_subscription(
            Bool, teleop_enable_topic, self._on_teleop_enable, 10
        )
        self.teleop_subscription = self.create_subscription(
            ArmJointAngles, teleop_topic, self._on_teleop_angles, 10
        )

        self.get_logger().info(
            f'回放直写: {playback_topic}；'
            f'遥操作: {teleop_topic}（由 {teleop_enable_topic} 使能）'
        )

    def _on_playback_angles(self, msg):
        """Replay angles with the original direct motion control."""
        if self._teleop_enabled:
            return

        values = [float(angle) for angle in msg.angles]
        if len(values) != 6:
            self.get_logger().warn(
                f'收到无效的回放角度数量: {len(values)}，期望 6 个'
            )
            return

        for servo_id, angle in zip(self.servo_ids, values):
            self.controller.enable_torque(servo_id)
            self.controller.set_servo_angle_direct(servo_id, angle)

    def _on_angles(self, msg):
        """Ignore visual catch angles while teleoperation is active."""
        if self._teleop_enabled:
            return
        super()._on_angles(msg)

    def _on_teleop_enable(self, msg):
        """Enable or disable leader-follower teleoperation."""
        enabled = bool(msg.data)
        if enabled == self._teleop_enabled:
            return
        self._teleop_enabled = enabled

        if enabled:
            # 取消视觉/回放可能正在执行的平滑轨迹
            self.controller.cancel_all_motions()
            self._teleop_positions = {}
            for servo_id in self.servo_ids:
                self._teleop_positions[servo_id] = (
                    self.controller.read_raw_position(servo_id)
                )
            self.get_logger().info('遥操作已开启，从动臂开始跟随领导臂')
        else:
            self._teleop_positions = {}
            self.get_logger().info('遥操作已停止，从动臂保持当前位置')

    def _on_teleop_angles(self, msg):
        """Follow the leader with a per-frame step limit (no jump on enable)."""
        if not self._teleop_enabled:
            return

        values = [float(angle) for angle in msg.angles]
        if len(values) != 6:
            self.get_logger().warn(
                f'收到无效的遥操作角度数量: {len(values)}，期望 6 个'
            )
            return
        if not all(math.isfinite(value) for value in values):
            return

        for servo_id, angle in zip(self.servo_ids, values):
            target = self.controller.angle_to_position(servo_id, angle)
            current = self._teleop_positions.get(servo_id)
            if current is None:
                current = self.controller.read_raw_position(servo_id)
                if current is None:
                    continue

            delta = target - current
            step = max(
                -self.teleop_max_step,
                min(self.teleop_max_step, delta),
            )
            position = current + step
            self._teleop_positions[servo_id] = position

            self.controller.enable_torque(servo_id)
            self.controller.write_position(servo_id, position)


def main(args=None):
    """Initialize ROS 2 and run the playback joint node."""
    rclpy.init(args=args)
    node = ArmPlaybackJointNode()
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
