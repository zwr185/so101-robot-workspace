"""On-demand action manager for voice-controlled arm actions.

语音工具不再常驻动作节点，而是向本节点发送请求：
  /robot_action/nod            Bool -> 启动 robot_nod_node 并触发点头
  /robot_action/shake_head     Bool -> 启动 robot_shake_head_node 并触发摇头
  /robot_action/grasp_box      Bool -> 启动抓取节点并触发视觉抓取
  /robot_action/remote_control Bool -> 启动/停止 leader_arm_joint 遥操作

同一时间只执行一个动作；新的动作会先停止上一个，避免节点或串口冲突。
"""

import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

NOD_DONE_TIMEOUT = 60.0
SHAKE_DONE_TIMEOUT = 60.0
GRASP_DONE_TIMEOUT = 120.0

DEFAULT_CALIBRATION = (
    '/home/zwr185/dev_ws/Seeed_RoboController/json/'
    'my_awesome_follower_arm.json'
)


class RobotActionManagerNode(Node):
    """Start action nodes only when the corresponding voice tool is called."""

    def __init__(self):
        """Create request subscriptions, triggers and process management state."""
        super().__init__('robot_action_manager_node')

        self.declare_parameter('startup_delay', 1.0)
        self.declare_parameter('grasp_startup_delay', 2.0)
        # 等待动作节点真正订阅触发话题的最长时间（秒）
        self.declare_parameter('startup_timeout', 10.0)
        self.declare_parameter('calibration_file', DEFAULT_CALIBRATION)
        self.declare_parameter('input_yaw_offset', -90.0)
        self.declare_parameter('grasp_offset_x', -2.0)
        self.declare_parameter('grasp_offset_y', 5.0)
        self.declare_parameter('video_device', '')
        self.declare_parameter('vision_idle_timeout', 30.0)
        self.declare_parameter('grasp_output_topic', '/arm_catch_angles')
        self.declare_parameter('grasp_feedback_topic', 'arm_joint_angles')

        self.startup_delay = float(
            self.get_parameter('startup_delay').value
        )
        self.grasp_startup_delay = float(
            self.get_parameter('grasp_startup_delay').value
        )
        self.startup_timeout = float(
            self.get_parameter('startup_timeout').value
        )
        self.calibration_file = self.get_parameter(
            'calibration_file').value
        self.input_yaw_offset = float(
            self.get_parameter('input_yaw_offset').value
        )
        self.grasp_offset_x = float(
            self.get_parameter('grasp_offset_x').value
        )
        self.grasp_offset_y = float(
            self.get_parameter('grasp_offset_y').value
        )
        self.video_device = self.get_parameter('video_device').value
        self.vision_idle_timeout = float(
            self.get_parameter('vision_idle_timeout').value
        )
        self.grasp_output_topic = self.get_parameter(
            'grasp_output_topic').value
        self.grasp_feedback_topic = self.get_parameter(
            'grasp_feedback_topic').value

        self._busy = False
        self._busy_lock = threading.Lock()
        self._processes = []
        self._leader_process = None
        self._teleop_enabled = False

        self.nod_done = threading.Event()
        self.shake_done = threading.Event()
        self.grasp_done = threading.Event()
        self.nod_result = False
        self.shake_result = False
        self.grasp_result = False

        self.create_subscription(
            Bool, '/robot_nod_done', self._on_nod_done, 10)
        self.create_subscription(
            Bool, '/robot_shake_head_done', self._on_shake_done, 10)
        self.create_subscription(
            Bool, '/robot_catch_box_done', self._on_grasp_done, 10)

        self.nod_trigger = self.create_publisher(Bool, '/robot_nod', 10)
        self.shake_trigger = self.create_publisher(
            Bool, '/robot_shake_head', 10)
        self.grasp_trigger = self.create_publisher(
            Bool, '/robot_catch_box', 10)
        self.remote_trigger = self.create_publisher(
            Bool, '/robot_remote_control', 10)

        self.create_subscription(
            Bool, '/robot_action/nod', self._on_nod_request, 10)
        self.create_subscription(
            Bool, '/robot_action/shake_head', self._on_shake_request, 10)
        self.create_subscription(
            Bool, '/robot_action/grasp_box', self._on_grasp_request, 10)
        self.create_subscription(
            Bool, '/robot_action/remote_control',
            self._on_remote_request, 10)

        self.get_logger().info(
            'robot_action_manager_node 已启动，动作节点按需拉起'
        )

    # --- 请求入口 ---

    def _on_nod_request(self, msg):
        if msg.data:
            self._run_one_shot('nod', self._nod_worker)

    def _on_shake_request(self, msg):
        if msg.data:
            self._run_one_shot('shake_head', self._shake_worker)

    def _on_grasp_request(self, msg):
        if msg.data:
            self._run_one_shot('grasp_box', self._grasp_worker)

    def _on_remote_request(self, msg):
        if msg.data:
            self._start_remote()
        else:
            self._stop_remote()

    # --- 完成信号 ---

    def _on_nod_done(self, msg):
        self.nod_result = bool(msg.data)
        self.nod_done.set()

    def _on_shake_done(self, msg):
        self.shake_result = bool(msg.data)
        self.shake_done.set()

    def _on_grasp_done(self, msg):
        self.grasp_result = bool(msg.data)
        self.grasp_done.set()

    # --- 一次性动作 ---

    def _run_one_shot(self, name, worker):
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn(
                    f'正在执行其他动作，忽略 {name}'
                )
                return
            self._busy = True

        if self._teleop_enabled:
            self._stop_remote()

        threading.Thread(
            target=self._guarded_worker,
            args=(name, worker),
            daemon=True,
        ).start()

    def _guarded_worker(self, name, worker):
        try:
            worker()
        except Exception as exc:
            self.get_logger().error(f'{name} 执行异常: {exc}')
        finally:
            self._terminate_processes(self._processes)
            self._processes = []
            with self._busy_lock:
                self._busy = False

    def _nod_worker(self):
        self.nod_done.clear()
        self.nod_result = False
        proc = self._start_process(
            ['ros2', 'run', 'ai_pkg', 'robot_nod_node']
        )
        if proc is None:
            return
        self._processes.append(proc)

        time.sleep(self.startup_delay)
        if not self._wait_for_trigger([proc], self.nod_trigger):
            self.get_logger().error(
                'robot_nod_node 启动失败或未就绪'
            )
            return

        self.nod_trigger.publish(Bool(data=True))
        if not self.nod_done.wait(NOD_DONE_TIMEOUT):
            self.get_logger().warn('等待点头完成超时')
        elif not self.nod_result:
            self.get_logger().warn('点头动作失败')

    def _shake_worker(self):
        self.shake_done.clear()
        self.shake_result = False
        proc = self._start_process(
            ['ros2', 'run', 'ai_pkg', 'robot_shake_head_node']
        )
        if proc is None:
            return
        self._processes.append(proc)

        time.sleep(self.startup_delay)
        if not self._wait_for_trigger([proc], self.shake_trigger):
            self.get_logger().error(
                'robot_shake_head_node 启动失败或未就绪'
            )
            return

        self.shake_trigger.publish(Bool(data=True))
        if not self.shake_done.wait(SHAKE_DONE_TIMEOUT):
            self.get_logger().warn('等待摇头完成超时')
        elif not self.shake_result:
            self.get_logger().warn('摇头动作失败')

    def _grasp_worker(self):
        self.grasp_done.clear()
        self.grasp_result = False
        arm_proc = self._start_process(self._grasp_arm_command())
        action_proc = self._start_process(self._grasp_action_command())
        if arm_proc is None or action_proc is None:
            self._terminate_processes(
                [proc for proc in (arm_proc, action_proc) if proc]
            )
            return
        self._processes.extend([arm_proc, action_proc])

        # 等待短延迟后确认两个进程都存活，且抓取节点已订阅触发话题，
        # 避免固定 sleep 结束后触发消息丢失导致视觉链路不启动
        time.sleep(self.grasp_startup_delay)
        if not self._wait_for_trigger(
            [arm_proc, action_proc], self.grasp_trigger
        ):
            self.get_logger().error(
                '抓取节点启动失败或未就绪（视觉链路未启动）'
            )
            return

        self.grasp_trigger.publish(Bool(data=True))
        if not self.grasp_done.wait(GRASP_DONE_TIMEOUT):
            self.get_logger().warn('等待抓取完成超时')
        elif self.grasp_result:
            self.get_logger().info('抓取完成')
        else:
            self.get_logger().warn(
                '抓取失败：未检测到目标或目标不可达'
            )

    def _wait_for_trigger(self, processes, publisher):
        """Wait until action processes are alive and subscribers are ready."""
        deadline = time.monotonic() + self.startup_timeout
        while time.monotonic() < deadline:
            if any(proc.poll() is not None for proc in processes):
                return False
            if publisher.get_subscription_count() > 0:
                return True
            time.sleep(0.1)
        return False

    # --- 遥操作 ---

    def _start_remote(self):
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn(
                    '正在执行其他动作，暂时无法开启遥操作'
                )
                return
        if self._teleop_enabled:
            return

        self._leader_process = self._start_process(
            ['ros2', 'run', 'remote_control_pkg', 'leader_arm_joint']
        )
        if self._leader_process is None:
            return

        self._teleop_enabled = True
        self.remote_trigger.publish(Bool(data=True))
        self.get_logger().info('遥操作已开启，领导臂节点已启动')

    def _stop_remote(self):
        if not self._teleop_enabled:
            return

        self._teleop_enabled = False
        self.remote_trigger.publish(Bool(data=False))
        if self._leader_process is not None:
            self._terminate_processes([self._leader_process])
            self._leader_process = None
        self.get_logger().info('遥操作已停止，领导臂节点已关闭')

    # --- 进程工具 ---

    def _start_process(self, command):
        try:
            return subprocess.Popen(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except Exception as exc:
            self.get_logger().error(
                f'启动命令失败 {command}: {exc}'
            )
            return None

    def _terminate_processes(self, processes):
        """Terminate action processes and their ros2 run children."""
        for proc in processes:
            if proc is not None and proc.poll() is None:
                self._signal_process_group(proc, signal.SIGTERM)

        # 给节点留出清理视觉子进程的时间，再强制结束
        deadline = time.time() + 5.0
        for proc in processes:
            if proc is None:
                continue
            timeout = max(0.1, deadline - time.time())
            try:
                proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                self._signal_process_group(proc, signal.SIGKILL)
                try:
                    proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    pass

    @staticmethod
    def _signal_process_group(proc, sig):
        """Send a signal to the process group without touching our own."""
        try:
            pgid = os.getpgid(proc.pid)
        except OSError:
            return

        if pgid == os.getpgid(0):
            # 防止误杀动作管理器自己的进程组
            try:
                proc.send_signal(sig)
            except OSError:
                pass
            return

        try:
            os.killpg(pgid, sig)
        except OSError:
            pass

    def _grasp_arm_command(self):
        return [
            'ros2', 'run', 'arm_pkg', 'arm_catch_box_node',
            '--ros-args',
            '-p', f'calibration_file:={self.calibration_file}',
            '-p', f'input_yaw_offset:={self.input_yaw_offset}',
            '-p', f'grasp_offset_x:={self.grasp_offset_x}',
            '-p', f'grasp_offset_y:={self.grasp_offset_y}',
            '-p', f'output_topic:={self.grasp_output_topic}',
            '-p', f'feedback_topic:={self.grasp_feedback_topic}',
            '-p', 'enable_keyboard:=true',
            '-p', 'enable_quit_key:=false',
            '-p', 'keyboard_only_when_busy:=true',
            '-p', 'verbose_motion:=false',
            '-p', 'rearm_delay:=0.0',
        ]

    def _grasp_action_command(self):
        command = [
            'ros2', 'run', 'ai_pkg', 'robot_catch_box_node',
            '--ros-args',
            '-p', f'vision_idle_timeout:={self.vision_idle_timeout}',
        ]
        if self.video_device:
            command.extend(['-p', f'video_device:={self.video_device}'])
        return command

    def destroy_node(self):
        """Stop teleoperation and all child action nodes on shutdown."""
        self._stop_remote()
        self._terminate_processes(self._processes)
        self._processes = []
        super().destroy_node()


def main(args=None):
    """Initialize ROS 2 and run the on-demand action manager."""
    rclpy.init(args=args)
    node = RobotActionManagerNode()
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
