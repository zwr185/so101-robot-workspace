"""MCP Server 节点：基于 FastMCP 提供机械臂动作工具。

运行方式：
  1) stdio（默认，供 MCP 客户端以子进程方式拉起）：
       ros2 run ai_pkg mcp_server_node
     在终端中手动运行时，可按 q 键退出。
  2) HTTP（便于调试，可配合 MCP Inspector 或浏览器）：
       ros2 run ai_pkg mcp_server_node --ros-args -p http:=true
     服务地址：http://127.0.0.1:8000/mcp

robot_nod 工具会向 /robot_action/nod 发布动作请求，
由 robot_action_manager_node 按需启动 robot_nod_node 并触发点头。

robot_shake_head 工具会向 /robot_action/shake_head 发布动作请求，
由 robot_action_manager_node 按需启动 robot_shake_head_node 并触发摇头。

robot_remote_control 工具会向 /robot_action/remote_control 发布请求，
由 robot_action_manager_node 按需启动/关闭 leader_arm_joint 遥操作。

robot_grasp_box 工具会向 /robot_action/grasp_box 发布动作请求，
由 robot_action_manager_node 按需启动抓取节点；结果通过
/robot_catch_box_done 返回。
"""

import os
import select
import sys
import termios
import threading
import tty

from fastmcp import FastMCP
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

# 创建 MCP 服务实例
mcp = FastMCP("机械臂控制服务")

# 等待 /robot_nod_done 完成信号的最长时间（秒）
NOD_TIMEOUT = 60.0

# 等待 /robot_shake_head_done 完成信号的最长时间（秒）
SHAKE_TIMEOUT = 60.0

# 等待 robot_catch_box_node 返回抓取结果的最长时间（秒）
GRASP_TIMEOUT = 100.0


def log(message):
    """日志统一写 stderr，避免污染 stdio 模式的协议输出通道。"""
    print(message, file=sys.stderr, flush=True)


def listen_quit_key():
    """stdio 模式下监听 q 键退出。

    MCP 协议占用了 stdin/stdout，所以按键从 /dev/tty 读取；
    只有在终端手动运行（stdin 是终端）时才启用，避免干扰客户端连接。
    """
    if not sys.stdin.isatty():
        return

    try:
        tty_fd = os.open("/dev/tty", os.O_RDONLY)
    except OSError:
        return

    old_settings = termios.tcgetattr(tty_fd)
    try:
        tty.setcbreak(tty_fd)  # 单键输入，无需回车
        while True:
            if select.select([tty_fd], [], [], 0.2)[0]:
                key = os.read(tty_fd, 1).decode(errors="ignore").lower()
                if key == "q":
                    termios.tcsetattr(tty_fd, termios.TCSADRAIN, old_settings)
                    # stdout 是 MCP 协议通道，提示信息必须写到 stderr
                    print("\n收到 q 键，MCP 服务已退出", file=sys.stderr, flush=True)
                    os._exit(0)
    finally:
        termios.tcsetattr(tty_fd, termios.TCSADRAIN, old_settings)
        os.close(tty_fd)


class McpServerNode(Node):
    """承载 MCP 服务的 ROS 2 节点。"""

    def __init__(self):
        super().__init__('mcp_server_node')
        self.declare_parameter('http', False)
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8000)
        # 动作请求由 robot_action_manager_node 按需拉起对应节点
        self.declare_parameter('grasp_trigger_topic', '/robot_action/grasp_box')
        self.declare_parameter('grasp_done_topic', '/robot_catch_box_done')
        self.declare_parameter(
            'shake_trigger_topic', '/robot_action/shake_head')
        self.declare_parameter('shake_done_topic', '/robot_shake_head_done')
        self.declare_parameter(
            'remote_control_topic', '/robot_action/remote_control')

        grasp_trigger_topic = self.get_parameter('grasp_trigger_topic').value
        grasp_done_topic = self.get_parameter('grasp_done_topic').value
        shake_trigger_topic = self.get_parameter('shake_trigger_topic').value
        shake_done_topic = self.get_parameter('shake_done_topic').value
        remote_control_topic = self.get_parameter(
            'remote_control_topic'
        ).value

        self.nod_publisher = self.create_publisher(
            Bool, '/robot_action/nod', 10)
        self.nod_done = threading.Event()
        self.create_subscription(
            Bool, '/robot_nod_done', self._on_nod_done, 10)

        self.shake_publisher = self.create_publisher(
            Bool, shake_trigger_topic, 10)
        self.shake_done = threading.Event()
        self.create_subscription(
            Bool, shake_done_topic, self._on_shake_done, 10)

        self.remote_control_publisher = self.create_publisher(
            Bool, remote_control_topic, 10)

        self.grasp_result = None
        self.grasp_done = threading.Event()
        self.grasp_publisher = self.create_publisher(
            Bool, grasp_trigger_topic, 10)
        self.create_subscription(
            Bool, grasp_done_topic, self._on_grasp_done, 10)

        mcp.add_tool(self.robot_nod)
        mcp.add_tool(self.robot_shake_head)
        mcp.add_tool(self.robot_remote_control)
        mcp.add_tool(self.robot_grasp_box)

    def _on_nod_done(self, msg):
        """Set the completion event when robot_nod_node finishes playback."""
        if msg.data:
            self.nod_done.set()

    def _on_shake_done(self, msg):
        """Set the completion event when robot_shake_head_node finishes."""
        if msg.data:
            self.shake_done.set()

    def _on_grasp_done(self, msg):
        """Record whether robot_catch_box_node completed the grasp."""
        self.grasp_result = bool(msg.data)
        self.grasp_done.set()

    def robot_nod(self) -> str:
        """控制机械臂执行点头动作，等待回放完成后返回。"""
        self.nod_done.clear()
        self.nod_publisher.publish(Bool(data=True))
        log("Executing robot_nod() -> published /robot_action/nod")

        if self.nod_done.wait(timeout=NOD_TIMEOUT):
            log("robot_nod() -> nod playback finished")
            return 'Robot arm nodded successfully.'

        log("robot_nod() -> timeout waiting for /robot_nod_done")
        return 'Robot arm nod command published, but no completion signal received.'

    def robot_shake_head(self) -> str:
        """控制机械臂执行摇头动作，等待回放完成后返回。"""
        self.shake_done.clear()
        self.shake_publisher.publish(Bool(data=True))
        log("Executing robot_shake_head() -> published /robot_action/shake_head")

        if self.shake_done.wait(timeout=SHAKE_TIMEOUT):
            log("robot_shake_head() -> shake playback finished")
            return 'Robot arm shook its head successfully.'

        log("robot_shake_head() -> timeout waiting for /robot_shake_head_done")
        return 'Robot arm shake command published, but no completion signal received.'

    def robot_remote_control(self, action: str = 'start') -> str:
        """开启或停止机械臂遥操作，action 取 start 或 stop。"""
        action = (action or 'start').strip().lower()
        if action not in ('start', 'stop'):
            return '参数错误：action 只能是 start 或 stop。'

        enable = action == 'start'
        self.remote_control_publisher.publish(Bool(data=enable))
        log(f'Executing robot_remote_control({action}) -> published {enable}')

        if enable:
            return (
                '遥操作已开启。请手动拖动领导臂，从动臂会跟随；'
                '说“停止遥操作”可以结束。'
            )
        return '遥操作已停止。'

    def robot_grasp_box(self) -> str:
        """视觉检测箱子位置，控制机械臂抓取并放到固定放置点。"""
        self.grasp_result = None
        self.grasp_done.clear()
        self.grasp_publisher.publish(Bool(data=True))
        log("Executing robot_grasp_box() -> published /robot_action/grasp_box")

        if not self.grasp_done.wait(timeout=GRASP_TIMEOUT):
            log('robot_grasp_box() -> timeout waiting for /robot_catch_box_done')
            return '抓取指令已发送，但等待抓取结果超时。'

        if self.grasp_result:
            log('robot_grasp_box() -> grasp finished')
            return 'Robot arm grasped and placed the box successfully.'

        log('robot_grasp_box() -> grasp failed')
        return '抓取失败：未检测到目标或目标超出工作空间，请调整箱子位置后重试。'

    def _spin_ros(self):
        """在后台线程处理 ROS 订阅回调。"""
        try:
            rclpy.spin(self)
        except ExternalShutdownException:
            pass

    def run(self):
        """阻塞运行 MCP 服务，直到进程退出。"""
        threading.Thread(target=self._spin_ros, daemon=True).start()

        if self.get_parameter('http').value:
            host = self.get_parameter('host').value
            port = self.get_parameter('port').value
            self.get_logger().info(f'HTTP 模式已启动：http://{host}:{port}/mcp')
            mcp.run(transport='http', host=host, port=port)
        else:
            self.get_logger().info(
                'stdio 模式已启动，等待 MCP 客户端连接；在终端中按 q 键可退出。')
            threading.Thread(target=listen_quit_key, daemon=True).start()
            mcp.run()


def main(args=None):
    rclpy.init(args=args)
    node = McpServerNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
