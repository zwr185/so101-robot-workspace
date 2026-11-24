"""Node executing a vision-guided grasp when /robot_catch_box is triggered.

本节点负责触发与流程衔接，实际抓取由 arm_pkg 的 arm_catch_box_node 执行：
  1. 收到 /robot_catch_box 触发后，按需启动相机、色块检测和坐标转换节点
  2. 等待视觉链路发布的新鲜 /box_world_pos_raw
  3. 把目标坐标转发到 /box_world_pos，触发 arm_catch_box_node 执行抓取
  4. 等待 /arm_catch_done，把结果发布到 /robot_catch_box_done
  5. 抓取结束且空闲超过 vision_idle_timeout 后关闭视觉链路，节省 CPU

视觉坐标与抓取触发分成两个话题（raw → world），避免相机持续检测时
机械臂反复自动抓取。

MCP Server 的 robot_grasp_box 工具通过 /robot_catch_box 触发本节点，
并等待 /robot_catch_box_done 获取本次抓取结果。
"""

from glob import glob
import os
import signal
import subprocess
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

# 等待抓取流程完成的最长时间（秒）
GRASP_TIMEOUT = 90.0


def _resolve_video_device():
    """Return a stable camera path or the first /dev/video* device."""
    stable_paths = sorted(glob('/dev/v4l/by-id/*-video-index0'))
    if stable_paths:
        # usb_cam 无法处理 by-id 软链接，这里解析成真实的 /dev/videoN
        return os.path.realpath(stable_paths[0])

    devices = sorted(glob('/dev/video*'))
    return devices[0] if devices else '/dev/video0'


class RobotCatchBoxNode(Node):
    """Trigger one vision-guided grasp and report the result."""

    def __init__(self):
        """Create topic connections, parameters and vision state."""
        super().__init__('robot_catch_box_node')
        self.declare_parameter('trigger_topic', '/robot_catch_box')
        self.declare_parameter('done_topic', '/robot_catch_box_done')
        self.declare_parameter('vision_topic', '/box_world_pos_raw')
        self.declare_parameter('target_topic', '/box_world_pos')
        self.declare_parameter('catch_done_topic', '/arm_catch_done')
        # 视觉链路按需启停
        self.declare_parameter('manage_vision', True)
        self.declare_parameter('video_device', '')
        self.declare_parameter('vision_idle_timeout', 30.0)
        self.declare_parameter('detect_timeout', 15.0)

        trigger_topic = self.get_parameter('trigger_topic').value
        done_topic = self.get_parameter('done_topic').value
        vision_topic = self.get_parameter('vision_topic').value
        target_topic = self.get_parameter('target_topic').value
        catch_done_topic = self.get_parameter('catch_done_topic').value

        self.manage_vision = bool(self.get_parameter('manage_vision').value)
        requested_device = str(
            self.get_parameter('video_device').value
        ).strip()
        self.video_device = requested_device or _resolve_video_device()
        self.vision_idle_timeout = float(
            self.get_parameter('vision_idle_timeout').value
        )
        self.detect_timeout = float(
            self.get_parameter('detect_timeout').value
        )

        vision_share = get_package_share_directory('vision_pkg')
        self.cam_config = os.path.join(vision_share, 'config', 'cam.yaml')
        self.detect_config = os.path.join(
            vision_share, 'config', 'box_detect.yaml'
        )
        self.tf_config = os.path.join(vision_share, 'config', 'box_tf.yaml')

        self.target_publisher = self.create_publisher(Point, target_topic, 10)
        self.done_publisher = self.create_publisher(Bool, done_topic, 10)
        self.subscription = self.create_subscription(
            Bool, trigger_topic, self._on_trigger, 10
        )
        self.vision_subscription = self.create_subscription(
            Point, vision_topic, self._on_vision_target, 10
        )
        self.catch_done_subscription = self.create_subscription(
            Bool, catch_done_topic, self._on_catch_done, 10
        )

        self.latest_target = None
        self.target_event = threading.Event()
        self.catch_result = None
        self.catch_done = threading.Event()
        self.busy = False

        # 视觉子进程与空闲回收状态
        self.vision_processes = []
        self.vision_lock = threading.Lock()
        self.last_vision_use = 0.0
        self.idle_timer = self.create_timer(1.0, self._on_idle_timer)

        if self.manage_vision:
            self.get_logger().info(
                f'robot_catch_box_node 已启动，视觉链路按需启停'
                f'（相机: {self.video_device}，'
                f'空闲 {self.vision_idle_timeout:.0f} 秒后关闭）'
            )
        else:
            self.get_logger().info(
                'robot_catch_box_node 已启动，manage_vision=false，'
                '使用外部启动的视觉链路'
            )

    def _on_vision_target(self, msg):
        """Remember the newest box position from the vision pipeline."""
        self.latest_target = (float(msg.x), float(msg.y), float(msg.z))
        self.target_event.set()

    def _on_catch_done(self, msg):
        """Record the grasp result published by arm_catch_box_node."""
        self.catch_result = bool(msg.data)
        self.catch_done.set()

    def _on_trigger(self, msg):
        """Start one grasp attempt without blocking the ROS executor."""
        if not msg.data:
            return
        if self.busy:
            self.get_logger().warn('正在抓取，忽略本次触发')
            return

        self.busy = True
        threading.Thread(
            target=self._run_grasp,
            name='robot_catch_box_worker',
            daemon=True,
        ).start()

    def _run_grasp(self):
        """Start vision if needed, forward one target, wait for completion."""
        success = False
        try:
            self._start_vision()
            # 丢弃旧目标，只使用本次触发之后检测到的坐标
            self.latest_target = None
            self.target_event.clear()
            if not self.target_event.wait(timeout=self.detect_timeout):
                self.get_logger().warn(
                    f'{self.detect_timeout:.1f} 秒内未检测到目标，'
                    '放弃本次抓取'
                )
                return

            x, y, z = self.latest_target
            self.catch_result = None
            self.catch_done.clear()
            self.target_publisher.publish(Point(x=x, y=y, z=z))
            self.get_logger().info(
                f'识别到物体 ({x:.1f}, {y:.1f}, {z:.1f})，开始抓取'
            )

            if not self.catch_done.wait(timeout=GRASP_TIMEOUT):
                self.get_logger().warn('等待 /arm_catch_done 超时')
                return

            success = bool(self.catch_result)
            if success:
                self.get_logger().info('抓取完成')
            else:
                self.get_logger().warn('目标被拒绝或抓取失败')
        finally:
            self.busy = False
            with self.vision_lock:
                self.last_vision_use = time.monotonic()
            self.done_publisher.publish(Bool(data=success))

    def _vision_alive(self):
        """Return True when all managed vision processes are still running."""
        return (
            len(self.vision_processes) == 3
            and all(proc.poll() is None for proc in self.vision_processes)
        )

    def _start_vision(self):
        """Start the camera, detection and coordinate nodes on demand."""
        if not self.manage_vision:
            return

        with self.vision_lock:
            if self._vision_alive():
                self.last_vision_use = time.monotonic()
                return

            self._stop_vision_locked()
            commands = [
                [
                    'ros2', 'run', 'usb_cam', 'usb_cam_node_exe',
                    '--ros-args', '--params-file', self.cam_config,
                    '-p', f'video_device:={self.video_device}',
                ],
                [
                    'ros2', 'run', 'vision_pkg', 'box_detect_node',
                    '--ros-args', '--params-file', self.detect_config,
                ],
                [
                    'ros2', 'run', 'vision_pkg', 'box_tf_node',
                    '--ros-args', '--params-file', self.tf_config,
                    '-p', 'output_topic:=/box_world_pos_raw',
                ],
            ]
            try:
                # start_new_session 让每个子进程独立成组，便于连同其
                # ros2 run 子进程一起结束
                self.vision_processes = [
                    subprocess.Popen(
                        command,
                        start_new_session=True,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                    for command in commands
                ]
            except OSError as exc:
                self.get_logger().error(f'启动视觉链路失败: {exc}')
                self.vision_processes = []
                return

            self.last_vision_use = time.monotonic()
            self.get_logger().info(
                '已启动视觉链路（相机 + 检测 + 坐标转换）'
            )

    def _stop_vision_locked(self):
        """Terminate all vision subprocesses; caller holds vision_lock."""
        if not self.vision_processes:
            return

        for proc in self.vision_processes:
            if proc.poll() is not None:
                continue
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except OSError:
                pass

        deadline = time.monotonic() + 2.0
        for proc in self.vision_processes:
            remaining = max(0.0, deadline - time.monotonic())
            try:
                proc.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except OSError:
                    pass

        self.vision_processes = []
        self.get_logger().info('视觉链路已关闭')

    def _on_idle_timer(self):
        """Stop the vision chain after it has been idle long enough."""
        if self.busy:
            return

        with self.vision_lock:
            if not self.vision_processes:
                return
            idle = time.monotonic() - self.last_vision_use
            if idle < self.vision_idle_timeout:
                return
            self._stop_vision_locked()

    def destroy_node(self):
        """Stop the vision chain before destroying the node."""
        with self.vision_lock:
            self._stop_vision_locked()
        super().destroy_node()


def main(args=None):
    """Initialize ROS 2 and run the vision grasp action node."""
    rclpy.init(args=args)
    node = RobotCatchBoxNode()
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
