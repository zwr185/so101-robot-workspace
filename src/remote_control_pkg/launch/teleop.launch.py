"""Launch leader and follower teleoperation, with p-to-quit on the terminal.

启动顺序：
  1. leader_arm_joint   —— 领导臂只读节点（断力矩，50Hz 发布角度）
  2. 1 秒后启动 follower_arm_joint —— 从动臂直写跟随节点

按 p 结束整个 launch，Ctrl+C 也可以。
两个机械臂节点不会打印逐帧关节角度，只保留必要的启动/状态日志。
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LEADER_PORT = (
    '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7B016147-if00'
)
FOLLOWER_PORT = (
    '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7B015685-if00'
)


def generate_launch_description():
    """Start the leader first, then the follower, with a p-key quit listener."""
    leader_port = LaunchConfiguration('leader_port')
    follower_port = LaunchConfiguration('follower_port')

    leader_node = Node(
        package='remote_control_pkg',
        executable='leader_arm_joint',
        name='leader_arm_joint',
        output='screen',
        parameters=[{'port': leader_port}],
    )

    follower_node = Node(
        package='remote_control_pkg',
        executable='follower_arm_joint',
        name='follower_arm_joint',
        output='screen',
        parameters=[{'port': follower_port}],
    )

    quit_key_node = Node(
        package='remote_control_pkg',
        executable='teleop_quit_key',
        name='teleop_quit_key',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'leader_port',
            default_value=LEADER_PORT,
            description='领导臂串口（固定 by-id 路径）',
        ),
        DeclareLaunchArgument(
            'follower_port',
            default_value=FOLLOWER_PORT,
            description='从动臂串口（固定 by-id 路径）',
        ),
        LogInfo(msg='遥操作：先启动领导臂，1 秒后启动从动臂'),
        LogInfo(msg='按 p 退出遥操作（Ctrl+C 也可以）'),
        RegisterEventHandler(
            OnProcessExit(
                target_action=quit_key_node,
                on_exit=[
                    LogInfo(msg='收到 p 键，正在关闭遥操作...'),
                    EmitEvent(event=Shutdown(reason='teleop quit key')),
                ],
            )
        ),
        leader_node,
        TimerAction(period=1.0, actions=[follower_node]),
        quit_key_node,
    ])
