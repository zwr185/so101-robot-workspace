"""Launch the arm playback driver together with the record/playback GUI.

录制回放使用 arm_playback_joint_node 直写目标位置，不做 1.2s 平滑，
保持原来的回放速度和录制节奏。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start the arm playback driver and the GUI record/playback node."""
    calibration_file = LaunchConfiguration('calibration_file')

    joint_node = Node(
        package='arm_pkg',
        executable='arm_playback_joint_node',
        name='arm_playback_joint_node',
        output='screen',
        parameters=[{
            'calibration_file': calibration_file,
            # cmd_angles 走直写（无平滑），用于录制回放
            'playback_topic': 'cmd_angles',
            # 录制与 GUI 订阅的是 arm_joint_angles，需与原来保持一致
            'feedback_topic': 'arm_joint_angles',
            # 与原来的 arm_joint_node 保持一致：10Hz 发布当前角度供录制
            'feedback_rate': 10.0,
        }],
    )

    gui_node = Node(
        package='arm_pkg',
        executable='gui_record_playback_node',
        name='gui_record_playback_node',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'calibration_file',
            default_value=(
                '/home/zwr185/dev_ws/Seeed_RoboController/json/'
                'my_awesome_follower_arm.json'
            ),
            description='LeRobot 校准文件路径，用于舵机角度映射',
        ),
        joint_node,
        gui_node,
    ])
