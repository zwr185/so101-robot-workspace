"""Node running a full pick-and-place sequence for the detected box."""

import atexit
import math
import os
import select
import sys
import termios
import threading
import time
import tty
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from arm_msg.msg import ArmJointAngles
from arm_pkg.arm_calibration import (
    joint_limits as calibration_joint_limits,
    load_calibration,
    raw_to_angle,
)
from arm_pkg.urdf_fk import load_model
from arm_pkg.urdf_ik import inverse_kinematics
from geometry_msgs.msg import Point
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

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


class ArmCatchBoxNode(Node):
    """Pick the detected box, then place it at a configured point."""

    def __init__(self):
        """Load the URDF model and create the subscriptions and publisher."""
        super().__init__('arm_catch_box_node')
        self.declare_parameter('input_topic', '/box_world_pos')
        self.declare_parameter('output_topic', '/arm_catch_angles')
        self.declare_parameter('feedback_topic', '/arm_current_angles')
        # 抓取结果信号：True=流程完成，False=目标被拒绝
        self.declare_parameter('done_topic', '/arm_catch_done')
        # 语音控制场景下关闭，避免和语音节点争抢 /dev/tty 按键
        self.declare_parameter('enable_keyboard', True)
        # q 键退出 launch 是否生效（语音场景关闭）
        self.declare_parameter('enable_quit_key', True)
        # 仅当抓取流程进行中才读取 /dev/tty 按键，
        # 空闲时把按键留给语音节点，避免互相抢输入
        self.declare_parameter('keyboard_only_when_busy', False)
        # 是否打印逐步运动日志（步骤 1/7 等），语音场景可关闭
        self.declare_parameter('verbose_motion', True)
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('target_link', 'gripper_frame_link')
        self.declare_parameter('pick_z_offset', 10.0)
        # 抓取时 TCP 相对球中心的高度（球半径附近），以及放置点上方的悬停高度
        self.declare_parameter('grasp_z_offset', 3.0)
        # 夹爪固定爪在中间，抓取点沿视觉/测量坐标系 x/y 轴偏移
        self.declare_parameter('grasp_offset_x', -2.0)
        self.declare_parameter('grasp_offset_y', 5.0)
        self.declare_parameter('place_clearance', 5.0)
        self.declare_parameter('place_x', -30.0)
        self.declare_parameter('place_y', 10.0)
        self.declare_parameter('place_z', 10.0)
        self.declare_parameter('joint_limit', 90.0)
        self.declare_parameter('calibration_file', '')
        # 视觉/测量坐标系到 URDF 底座系的 yaw 旋转（度），绕 z 轴
        self.declare_parameter('input_yaw_offset', 0.0)
        # 夹爪开/合使用舵机原始位置，便于在校准文件变化后保持物理位置不变
        self.declare_parameter('gripper_open_raw', 3072.0)
        self.declare_parameter('gripper_close_raw', 2048.0)
        self.declare_parameter('step_delay', 2.0)
        self.declare_parameter('rearm_delay', 5.0)
        # 与 arm_track_box_node 一致：平滑接近
        self.declare_parameter('approach_alpha', 0.3)
        self.declare_parameter('approach_interval', 0.5)
        self.declare_parameter('approach_tolerance', 1.0)
        self.declare_parameter('ik_restarts', 4)
        self.declare_parameter('ik_max_iterations', 300)
        self.declare_parameter('ik_tolerance', 0.1)

        self.input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        feedback_topic = self.get_parameter('feedback_topic').value
        done_topic = self.get_parameter('done_topic').value
        self.enable_keyboard = bool(
            self.get_parameter('enable_keyboard').value
        )
        self.enable_quit_key = bool(
            self.get_parameter('enable_quit_key').value
        )
        self.keyboard_only_when_busy = bool(
            self.get_parameter('keyboard_only_when_busy').value
        )
        self.verbose_motion = bool(
            self.get_parameter('verbose_motion').value
        )
        urdf_path = (
            self.get_parameter('urdf_path').value or str(_default_urdf_path())
        )
        self.target_link = self.get_parameter('target_link').value
        self.pick_z_offset = float(
            self.get_parameter('pick_z_offset').value
        )
        self.grasp_z_offset = float(
            self.get_parameter('grasp_z_offset').value
        )
        self.grasp_offset_x = float(
            self.get_parameter('grasp_offset_x').value
        )
        self.grasp_offset_y = float(
            self.get_parameter('grasp_offset_y').value
        )
        self.place_clearance = float(
            self.get_parameter('place_clearance').value
        )
        self.place_point = [
            float(self.get_parameter('place_x').value),
            float(self.get_parameter('place_y').value),
            float(self.get_parameter('place_z').value),
        ]
        self.joint_limit = abs(float(self.get_parameter('joint_limit').value))
        calibration_file = self.get_parameter('calibration_file').value
        calibration = load_calibration(calibration_file)
        self.input_yaw_offset = float(
            self.get_parameter('input_yaw_offset').value
        )
        yaw = math.radians(self.input_yaw_offset)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        self.place_point_base = [
            self.place_point[0] * cos_yaw - self.place_point[1] * sin_yaw,
            self.place_point[0] * sin_yaw + self.place_point[1] * cos_yaw,
            self.place_point[2],
        ]

        gripper_entry = calibration.get('gripper')
        open_raw = float(self.get_parameter('gripper_open_raw').value)
        close_raw = float(self.get_parameter('gripper_close_raw').value)
        if gripper_entry is not None:
            self.gripper_open = raw_to_angle(gripper_entry, open_raw)
            self.gripper_close = raw_to_angle(gripper_entry, close_raw)
        else:
            self.gripper_open = (open_raw - 2048.0) / 1024.0 * 90.0
            self.gripper_close = (close_raw - 2048.0) / 1024.0 * 90.0
        self.step_delay = float(self.get_parameter('step_delay').value)
        self.rearm_delay = float(self.get_parameter('rearm_delay').value)
        self.approach_alpha = float(
            self.get_parameter('approach_alpha').value
        )
        self.approach_interval = float(
            self.get_parameter('approach_interval').value
        )
        self.approach_tolerance = float(
            self.get_parameter('approach_tolerance').value
        )
        self.ik_restarts = int(self.get_parameter('ik_restarts').value)
        self.ik_max_iterations = int(
            self.get_parameter('ik_max_iterations').value
        )
        self.ik_tolerance = float(self.get_parameter('ik_tolerance').value)

        self.model = load_model(urdf_path)
        self.joint_names = self.model.movable_joint_names(self.target_link)
        self.limits = self._build_joint_limits(calibration)

        self.sequence = []
        self.step_index = 0
        self.step_deadline = 0.0
        self.busy = False
        self.rearm_time = 0.0
        self.last_joints = None
        self.subscription = None

        self.phase = 'idle'
        self.approach_joints = None
        self.approach_goal = None
        self.approach_last_time = 0.0
        self.current_joints = None

        self.paused = False
        self.pause_start = None
        self.last_gripper = self.gripper_open
        self._stdin_fd = None
        self._opened_tty = False
        self._termios_settings = None
        self._input_stop = threading.Event()
        self._input_thread = None

        self.publisher_ = self.create_publisher(
            ArmJointAngles, output_topic, 10
        )
        self.done_publisher = self.create_publisher(Bool, done_topic, 10)
        self.feedback_subscription = self.create_subscription(
            ArmJointAngles, feedback_topic, self._on_current_angles, 10
        )
        self._subscribe_targets()
        self.timer = self.create_timer(0.05, self._on_timer)

        self.get_logger().info(
            f'arm_catch_box_node 已启动，订阅: {self.input_topic}，发布: '
            f'{output_topic}，位姿反馈: {feedback_topic}'
        )
        if self.grasp_z_offset >= self.pick_z_offset:
            self.get_logger().warn(
                'grasp_z_offset >= pick_z_offset，抓取阶段不会低于悬停点，'
                '请检查配置'
            )
        if self.enable_keyboard:
            keys = 's=停止抓取, g=重新抓取'
            if self.enable_quit_key:
                keys += ', q=退出 launch'
            self.get_logger().info(f'按键控制: {keys}')
            self._start_keyboard()
        else:
            self.get_logger().info(
                '已禁用终端按键控制（enable_keyboard=false）'
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

    def _on_current_angles(self, msg):
        """Remember the measured pose for approach start and pause hold."""
        if len(msg.angles) < len(self.joint_names):
            return
        self.current_joints = {
            name: float(value)
            for name, value in zip(self.joint_names, msg.angles)
        }

    def _subscribe_targets(self):
        """Start accepting target coordinates."""
        if self.subscription is None:
            self.subscription = self.create_subscription(
                Point, self.input_topic, self._on_box_point, 10
            )

    def _unsubscribe_targets(self):
        """Stop accepting target coordinates during the sequence."""
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None

    def _on_box_point(self, msg):
        """Remember the box position and start the pick-and-place sequence."""
        if self.busy or self.paused:
            return

        now = time.monotonic()
        if now < self.rearm_time:
            return

        yaw = math.radians(self.input_yaw_offset)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        input_x = float(msg.x)
        input_y = float(msg.y)
        # 偏移量在视觉/测量坐标系里调节，再一起旋转到底座系
        grasp_input_x = input_x + self.grasp_offset_x
        grasp_input_y = input_y + self.grasp_offset_y
        ball_x = grasp_input_x * cos_yaw - grasp_input_y * sin_yaw
        ball_y = grasp_input_x * sin_yaw + grasp_input_y * cos_yaw
        ball_z = float(msg.z)
        hover_target = [ball_x, ball_y, ball_z + self.pick_z_offset]
        grasp_target = [ball_x, ball_y, ball_z + self.grasp_z_offset]
        release_target = list(self.place_point_base)
        place_hover_target = [
            self.place_point_base[0],
            self.place_point_base[1],
            self.place_point_base[2] + self.place_clearance,
        ]

        start_joints = self.current_joints or self.last_joints
        hover_joints, hover_error = inverse_kinematics(
            self.model, hover_target, self.target_link,
            initial=start_joints, limits=self.limits,
            restarts=self.ik_restarts,
            max_iterations=self.ik_max_iterations,
            tolerance=self.ik_tolerance,
        )
        if hover_error > MAX_POSITION_ERROR_CM:
            self.get_logger().warn(
                f'悬停点反解误差 {hover_error:.2f} cm，'
                f'超出工作空间，忽略本次目标'
            )
            self._publish_done(False)
            return

        grasp_joints, grasp_error = inverse_kinematics(
            self.model, grasp_target, self.target_link,
            initial=hover_joints, limits=self.limits,
            restarts=self.ik_restarts,
            max_iterations=self.ik_max_iterations,
            tolerance=self.ik_tolerance,
        )
        if grasp_error > MAX_POSITION_ERROR_CM:
            self.get_logger().warn(
                f'抓取点反解误差 {grasp_error:.2f} cm，'
                f'检查 grasp_z_offset 是否合理'
            )
            self._publish_done(False)
            return

        release_joints, release_error = inverse_kinematics(
            self.model, release_target, self.target_link,
            initial=grasp_joints, limits=self.limits,
            restarts=self.ik_restarts,
            max_iterations=self.ik_max_iterations,
            tolerance=self.ik_tolerance,
        )
        if release_error > MAX_POSITION_ERROR_CM:
            self.get_logger().warn(
                f'放置点反解误差 {release_error:.2f} cm，'
                f'请检查 place_x/place_y/place_z 配置'
            )
            self._publish_done(False)
            return

        place_hover_joints, place_hover_error = inverse_kinematics(
            self.model, place_hover_target, self.target_link,
            initial=release_joints, limits=self.limits,
            restarts=self.ik_restarts,
            max_iterations=self.ik_max_iterations,
            tolerance=self.ik_tolerance,
        )
        if place_hover_error > MAX_POSITION_ERROR_CM:
            self.get_logger().warn(
                f'放置悬停点反解误差 {place_hover_error:.2f} cm，'
                f'请检查 place_clearance'
            )
            self._publish_done(False)
            return

        if start_joints is None:
            start_joints = {name: 0.0 for name in self.joint_names}

        # 参考 arm_track_box_node 的平滑节奏：从当前实际角度出发，按比例
        # 逼近抓取位姿。关节空间插值可避免笛卡尔直线穿过不可达区域时
        # IK 跳到另一个解分支造成的突然大角度甩动
        self.approach_joints = {
            name: float(start_joints.get(name, 0.0))
            for name in self.joint_names
        }
        self.approach_goal = dict(hover_joints)
        self.approach_last_time = 0.0
        self.last_joints = self.approach_joints
        self.phase = 'approach'

        self.sequence = [
            (grasp_joints, self.gripper_open, '下降到抓取高度'),
            (grasp_joints, self.gripper_close, '闭合夹爪'),
            (hover_joints, self.gripper_close, '抬起'),
            (place_hover_joints, self.gripper_close, '移动到放置点上方'),
            (release_joints, self.gripper_close, '下降到放置高度'),
            (release_joints, self.gripper_open, '张开夹爪放球'),
            (place_hover_joints, self.gripper_open, '抬起'),
        ]
        self.step_index = 0
        self.step_deadline = now
        self.busy = True

        # 流程执行期间不再接收新的目标坐标
        self._unsubscribe_targets()

        self._log_motion(
            '收到目标，开始平滑接近并执行下降-夹取-抬起-放置流程'
        )

    def _on_timer(self):
        """Publish the next approach point or sequence step."""
        if not self.busy or self.paused:
            return

        now = time.monotonic()

        if self.phase == 'approach':
            self._on_approach_tick(now)
            return

        if now < self.step_deadline:
            return

        if self.step_index >= len(self.sequence):
            self.busy = False
            self.phase = 'idle'
            self.rearm_time = now + self.rearm_delay
            self.sequence = []
            self._subscribe_targets()
            self._log_motion(
                f'抓取放置流程完成，{self.rearm_delay:.1f} 秒内忽略新目标'
            )
            self._publish_done(True)
            return

        joints, gripper, label = self.sequence[self.step_index]
        self._publish_angles(joints, gripper)
        self.last_joints = joints

        self.step_index += 1
        self.step_deadline = now + self.step_delay
        self._log_motion(
            f'步骤 {self.step_index}/{len(self.sequence)}: {label}'
        )

    def _on_approach_tick(self, now):
        """Blend the joint angles towards the pick pose like the tracking."""
        if now - self.approach_last_time < self.approach_interval:
            return
        self.approach_last_time = now

        alpha = self.approach_alpha
        joints = {
            name: alpha * self.approach_goal[name]
            + (1.0 - alpha) * self.approach_joints[name]
            for name in self.joint_names
        }
        self.approach_joints = joints

        self._publish_angles(joints, self.gripper_open)
        self.last_joints = joints

        max_difference = max(
            abs(joints[name] - self.approach_goal[name])
            for name in self.joint_names
        )
        if max_difference <= self.approach_tolerance:
            self.phase = 'sequence'
            self.step_index = 0
            self.step_deadline = now + self.approach_interval
            self._log_motion(
                '已到达抓取点上方，开始执行抓取放置流程'
            )

    def _start_keyboard(self):
        """Start the terminal key listener when a controlling tty exists."""
        fd = None
        try:
            fd = os.open('/dev/tty', os.O_RDONLY)
            self._opened_tty = True
        except OSError:
            try:
                if sys.stdin is not None and sys.stdin.isatty():
                    fd = sys.stdin.fileno()
            except (AttributeError, OSError, ValueError):
                fd = None

        if fd is None or not os.isatty(fd):
            if fd is not None and self._opened_tty:
                os.close(fd)
                self._opened_tty = False
            self.get_logger().warn(
                '当前终端不支持按键控制，s/g/q 不可用'
            )
            return

        self._stdin_fd = fd
        self._input_thread = threading.Thread(
            target=self._keyboard_loop,
            name='arm_catch_keyboard',
            daemon=True,
        )
        self._input_thread.start()

    def _keyboard_loop(self):
        """Read single key presses from the controlling terminal."""
        fd = self._stdin_fd
        try:
            self._termios_settings = termios.tcgetattr(fd)
            tty.setcbreak(fd, termios.TCSANOW)
        except (OSError, termios.error) as exc:
            self.get_logger().warn(f'无法设置终端按键模式: {exc}')
            return

        atexit.register(self._restore_terminal)
        busy_seen = False
        try:
            while rclpy.ok() and not self._input_stop.is_set():
                if self.keyboard_only_when_busy:
                    if not self.busy:
                        # 空闲时不读 /dev/tty，把按键留给语音节点
                        busy_seen = False
                        time.sleep(0.1)
                        continue
                    if not busy_seen:
                        # 丢弃空闲期间残留的按键，避免误停止/误重新抓取
                        try:
                            termios.tcflush(fd, termios.TCIFLUSH)
                        except (OSError, termios.error):
                            pass
                        busy_seen = True

                ready, _, _ = select.select([fd], [], [], 0.2)
                if not ready:
                    continue

                data = os.read(fd, 1)
                if not data:
                    break

                key = data.decode(errors='ignore').lower()
                if key == 's':
                    self._stop_grab()
                elif key == 'g':
                    self._restart_grab()
                elif key == 'q':
                    if not self.enable_quit_key:
                        continue
                    self.get_logger().info(
                        '收到退出按键 q，正在退出 launch'
                    )
                    self._input_stop.set()
                    rclpy.shutdown()
                    break
        finally:
            self._restore_terminal()

    def _stop_grab(self):
        """Stop the current motion and keep the arm at its measured pose."""
        if self.paused:
            self.get_logger().info('已经处于停止状态')
            return

        self.paused = True
        self.pause_start = time.monotonic()
        hold_joints = self.current_joints or self.last_joints
        if self.busy and hold_joints is not None:
            # 用最新反馈角度覆盖当前目标，尽量让舵机立刻停住
            self._publish_angles(hold_joints, self.last_gripper)
        self.get_logger().info(
            '已停止抓取，机械臂保持当前位置；按 g 重新抓取'
        )

    def _restart_grab(self):
        """Abort the current action and wait for a fresh visual target."""
        self.paused = True
        self.pause_start = None
        hold_joints = self.current_joints or self.last_joints
        if self.busy and hold_joints is not None:
            self._publish_angles(hold_joints, self.last_gripper)

        self.busy = False
        self.phase = 'idle'
        self.sequence = []
        self.step_index = 0
        self.step_deadline = 0.0
        self.rearm_time = 0.0
        self.approach_joints = None
        self.approach_goal = None
        self.approach_last_time = 0.0
        self.paused = False
        self._subscribe_targets()
        self.get_logger().info(
            '重新抓取：已中止当前动作，等待新的视觉目标'
        )

    def _restore_terminal(self):
        """Restore the terminal mode and close an opened /dev/tty."""
        fd = self._stdin_fd
        if self._termios_settings is not None and fd is not None:
            try:
                termios.tcsetattr(
                    fd, termios.TCSADRAIN, self._termios_settings
                )
            except (OSError, termios.error):
                pass
        self._termios_settings = None

        if self._opened_tty and fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass
            self._stdin_fd = None
            self._opened_tty = False

    def _log_motion(self, message):
        """Log grasp progress only when verbose motion logs are enabled."""
        if self.verbose_motion:
            self.get_logger().info(message)

    def _publish_done(self, success):
        """Publish the grasp result for the MCP tool to wait on."""
        self.done_publisher.publish(Bool(data=success))

    def _publish_angles(self, joints, gripper):
        """Publish five joint angles plus the gripper angle."""
        angles = [joints[name] for name in self.joint_names]
        angles.append(gripper)
        self.last_gripper = float(gripper)

        cmd = ArmJointAngles()
        cmd.angles = [float(angle) for angle in angles]
        self.publisher_.publish(cmd)

    def destroy_node(self):
        """Stop the key listener and restore the terminal before exit."""
        self._input_stop.set()
        if self._input_thread is not None and self._input_thread.is_alive():
            self._input_thread.join(timeout=0.5)
        self._restore_terminal()
        super().destroy_node()


def main(args=None):
    """Initialize ROS 2 and run the pick-and-place node."""
    rclpy.init(args=args)
    node = ArmCatchBoxNode()
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
