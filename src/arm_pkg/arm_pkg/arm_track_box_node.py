"""Node tracking the detected box position with inverse kinematics."""

import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from arm_msg.msg import ArmJointAngles
from arm_pkg.arm_calibration import (
    joint_limits as calibration_joint_limits,
    load_calibration,
)
from arm_pkg.urdf_fk import load_model
from arm_pkg.urdf_ik import inverse_kinematics
from geometry_msgs.msg import Point
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

MAX_POSITION_ERROR_CM = 2.0


def _default_urdf_path():
    """Resolve the arm URDF from the installed share directory."""
    try:
        share_dir = Path(get_package_share_directory('arm_pkg'))
        candidate = share_dir / 'urdf' / 'arm.urdf.xacro'
        if candidate.exists():
            return candidate
    except Exception:
        pass

    source_dir = Path(__file__).resolve().parents[1]
    return source_dir / 'urdf' / 'arm.urdf.xacro'


class ArmTrackBoxNode(Node):
    """Solve IK for the box position and publish the joint angles."""

    def __init__(self):
        """Load the URDF model and create the subscriptions and publisher."""
        super().__init__('arm_track_box_node')
        self.declare_parameter('input_topic', '/box_world_pos')
        self.declare_parameter('output_topic', '/arm_catch_angles')
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('target_link', 'gripper_frame_link')
        self.declare_parameter('solve_interval', 0.5)
        self.declare_parameter('z_offset', 20.0)
        self.declare_parameter('smooth_alpha', 0.3)
        self.declare_parameter('min_target_change', 0.5)
        self.declare_parameter('joint_limit', 90.0)
        self.declare_parameter('calibration_file', '')
        # 视觉/测量坐标系到 URDF 底座系的 yaw 旋转（度），绕 z 轴
        self.declare_parameter('input_yaw_offset', 0.0)
        # 单次热启动容易困在关节限位处的局部极小值，需要少量随机重启
        self.declare_parameter('ik_restarts', 4)
        self.declare_parameter('ik_max_iterations', 300)
        self.declare_parameter('ik_tolerance', 0.1)
        self.declare_parameter('failed_target_cooldown', 2.0)
        self.declare_parameter('failed_target_radius', 2.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        urdf_path = (
            self.get_parameter('urdf_path').value or str(_default_urdf_path())
        )
        self.target_link = self.get_parameter('target_link').value
        self.solve_interval = float(
            self.get_parameter('solve_interval').value
        )
        self.z_offset = float(self.get_parameter('z_offset').value)
        self.smooth_alpha = float(self.get_parameter('smooth_alpha').value)
        self.min_target_change = float(
            self.get_parameter('min_target_change').value
        )
        self.joint_limit = abs(float(self.get_parameter('joint_limit').value))
        calibration_file = self.get_parameter('calibration_file').value
        calibration = load_calibration(calibration_file)
        self.input_yaw_offset = float(
            self.get_parameter('input_yaw_offset').value
        )
        self.ik_restarts = int(self.get_parameter('ik_restarts').value)
        self.ik_max_iterations = int(
            self.get_parameter('ik_max_iterations').value
        )
        self.ik_tolerance = float(self.get_parameter('ik_tolerance').value)
        self.failed_cooldown = float(
            self.get_parameter('failed_target_cooldown').value
        )
        self.failed_radius = float(
            self.get_parameter('failed_target_radius').value
        )
        self.last_solve_time = 0.0

        self.smoothed_target = None
        self.last_joints = None
        self.last_command = None
        self.failed_target = None
        self.failed_time = 0.0

        self.model = load_model(urdf_path)
        self.joint_names = self.model.movable_joint_names(self.target_link)
        self.limits = self._build_joint_limits(calibration)
        limits_text = ', '.join(
            f'{name}[{lower:.1f}, {upper:.1f}]'
            for name, (lower, upper) in self.limits.items()
        )

        self.publisher_ = self.create_publisher(
            ArmJointAngles, output_topic, 10
        )
        self.subscription = self.create_subscription(
            Point, input_topic, self._on_box_point, 10
        )
        self.get_logger().info(
            f'arm_track_box_node 已启动，订阅: {input_topic}，发布: '
            f'{output_topic}，URDF: {urdf_path}，目标链接: {self.target_link}，'
            f'Z 轴抬升: {self.z_offset} cm，IK 容差: {self.ik_tolerance} cm\n'
            f'输入坐标 yaw 偏置: {self.input_yaw_offset} 度\n'
            f'关节顺序: {self.joint_names}\n'
            f'校准文件: {calibration_file or "未使用"}\n'
            f'关节限位 (度): {limits_text}'
        )

    def _build_joint_limits(self, calibration):
        """Merge calibrated limits with the URDF limits for the IK solver."""
        calibrated = calibration_joint_limits(
            calibration, self.joint_names
        )
        limits = {}
        for name in self.joint_names:
            joint = self.model.joints[name]
            urdf_lower = joint.lower if joint.lower is not None else -180.0
            urdf_upper = joint.upper if joint.upper is not None else 180.0
            if name in calibrated:
                lower = max(urdf_lower, calibrated[name][0])
                upper = min(urdf_upper, calibrated[name][1])
            else:
                lower = -self.joint_limit
                upper = self.joint_limit
            limits[name] = (lower, upper)
        return limits

    def _on_box_point(self, msg):
        """Solve IK for the lifted target and publish the joint angles."""
        now = time.monotonic()
        if now - self.last_solve_time < self.solve_interval:
            return
        self.last_solve_time = now

        yaw = math.radians(self.input_yaw_offset)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        input_x = float(msg.x)
        input_y = float(msg.y)
        base_x = input_x * cos_yaw - input_y * sin_yaw
        base_y = input_x * sin_yaw + input_y * cos_yaw
        raw_target = [
            base_x,
            base_y,
            float(msg.z) + self.z_offset,
        ]
        if self.smoothed_target is None:
            self.smoothed_target = raw_target
        else:
            alpha = self.smooth_alpha
            self.smoothed_target = [
                alpha * new + (1.0 - alpha) * old
                for new, old in zip(raw_target, self.smoothed_target)
            ]
        target = [round(value, 3) for value in self.smoothed_target]

        if self.last_command is not None:
            if math.dist(target, self.last_command) < self.min_target_change:
                return

        # 同一个不可达目标在冷却期内不重复求解，避免占满 CPU
        if self.failed_target is not None:
            recently_failed = now - self.failed_time < self.failed_cooldown
            near_failed = (
                math.dist(target, self.failed_target) < self.failed_radius
            )
            if recently_failed and near_failed:
                return

        joints, error = inverse_kinematics(
            self.model, target, self.target_link, initial=self.last_joints,
            limits=self.limits, restarts=self.ik_restarts,
            max_iterations=self.ik_max_iterations,
            tolerance=self.ik_tolerance,
        )

        if error > MAX_POSITION_ERROR_CM:
            self.failed_target = target
            self.failed_time = now
            self.get_logger().warn(
                f'目标位置 (cm): {target}，位置误差较大 ({error:.2f} cm)，'
                f'目标可能超出工作空间，不发送角度指令',
                throttle_duration_sec=2.0,
            )
            return

        self.failed_target = None

        lines = '\n'.join(
            f'  {name:14s} {value: .2f} 度'
            for name, value in joints.items()
        )
        self.get_logger().info(
            f'目标位置 (cm): {target}，位置误差: {error:.2f} cm\n{lines}'
        )

        angles = [joints[name] for name in self.joint_names]
        angles.append(0.0)

        cmd = ArmJointAngles()
        cmd.angles = [float(angle) for angle in angles]
        self.publisher_.publish(cmd)

        self.last_joints = joints
        self.last_command = target


def main(args=None):
    """Initialize ROS 2 and run the arm track box node."""
    rclpy.init(args=args)
    node = ArmTrackBoxNode()
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
