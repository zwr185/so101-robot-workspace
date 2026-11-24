"""语音控制 launch：常驻核心节点 + 语音触发时按需启动动作节点。

常驻节点：
  1. arm_pkg/arm_playback_joint_node —— 从动臂唯一串口驱动
     （处理 cmd_angles 回放、/arm_catch_angles 视觉、/leader_joint_angles 遥操作）
  2. ai_pkg/robot_remote_control_node —— 遥操作使能开关（/teleop_enable）
  3. ai_pkg/robot_action_manager_node —— 按需拉起/关闭动作节点
  4. ai_pkg/llm_agent_node —— 语音识别 + 大模型决策 + MCP

按需节点（由 robot_action_manager_node 在语音工具调用时启动）：
  - robot_nod_node              —— 点头回放
  - robot_shake_head_node       —— 摇头回放
  - arm_catch_box_node + robot_catch_box_node —— 视觉抓取
  - leader_arm_joint            —— 遥操作领导臂

这样动作节点只在需要时占用 CPU 和串口，后续新增工具也按同一模式扩展。
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context):
    """Build the always-on core nodes; action nodes are started on demand."""
    video_device = LaunchConfiguration('video_device').perform(
        context
    ).strip()
    vision_idle_timeout = float(
        LaunchConfiguration('vision_idle_timeout').perform(context).strip()
    )
    calibration_file = LaunchConfiguration('calibration_file').perform(
        context
    ).strip()
    input_yaw_offset = float(
        LaunchConfiguration('input_yaw_offset').perform(context).strip()
    )
    grasp_offset_x = float(
        LaunchConfiguration('grasp_offset_x').perform(context).strip()
    )
    grasp_offset_y = float(
        LaunchConfiguration('grasp_offset_y').perform(context).strip()
    )

    # 从动臂唯一串口驱动：三条控制路径共用同一个端口
    joint_node = Node(
        package='arm_pkg',
        executable='arm_playback_joint_node',
        name='arm_playback_joint_node',
        output='screen',
        parameters=[{
            'input_topic': '/arm_catch_angles',
            'playback_topic': 'cmd_angles',
            'feedback_topic': 'arm_joint_angles',
            'teleop_topic': '/leader_joint_angles',
            'teleop_enable_topic': '/teleop_enable',
            'calibration_file': calibration_file,
        }],
    )

    # 遥操作使能开关：把 /robot_remote_control 转成 /teleop_enable
    remote_control_node = Node(
        package='ai_pkg',
        executable='robot_remote_control_node',
        name='robot_remote_control_node',
        output='screen',
    )

    # 动作管理器：收到语音工具请求后按需启动对应节点
    action_manager_node = Node(
        package='ai_pkg',
        executable='robot_action_manager_node',
        name='robot_action_manager_node',
        output='screen',
        parameters=[{
            'calibration_file': calibration_file,
            'input_yaw_offset': input_yaw_offset,
            'grasp_offset_x': grasp_offset_x,
            'grasp_offset_y': grasp_offset_y,
            'video_device': video_device,
            'vision_idle_timeout': vision_idle_timeout,
            'grasp_output_topic': '/arm_catch_angles',
            'grasp_feedback_topic': 'arm_joint_angles',
        }],
    )

    agent_node = Node(
        package='ai_pkg',
        executable='llm_agent_node',
        name='llm_agent_node',
        output='screen',
    )

    return [
        LogInfo(
            msg='常驻节点: 从动臂驱动、遥操作开关、动作管理器、语音 Agent'
        ),
        LogInfo(
            msg='动作节点按需启动: 点头 / 摇头 / 视觉抓取 / 遥操作'
        ),
        LogInfo(msg=f'相机设备: {video_device or "自动检测"}'),
        LogInfo(msg=f'视觉坐标→底座坐标 yaw 旋转: {input_yaw_offset} 度'),
        LogInfo(
            msg='语音提示: r=录音, s=停止录音/停止抓取, g=重新抓取, '
            'q=退出程序；说“点个头/摇个头/抓取箱子/抓取球/抓取物体/'
            '开启遥操作”触发对应动作'
        ),
        joint_node,
        remote_control_node,
        action_manager_node,
        agent_node,
    ]


def generate_launch_description():
    """Start the always-on voice control core nodes."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value='',
            description='相机设备路径，留空则自动检测 /dev/v4l/by-id 下的稳定链接',
        ),
        DeclareLaunchArgument(
            'vision_idle_timeout',
            default_value='30.0',
            description='抓取结束后视觉链路保持运行的秒数，超时后自动关闭',
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=(
                '/home/zwr185/dev_ws/Seeed_RoboController/json/'
                'my_awesome_follower_arm.json'
            ),
            description='LeRobot 校准文件路径，用于舵机角度映射和关节限位',
        ),
        DeclareLaunchArgument(
            'input_yaw_offset',
            default_value='-90.0',
            description='视觉/测量坐标绕底座 z 轴的旋转角（度），转换到 URDF 底座系',
        ),
        DeclareLaunchArgument(
            'grasp_offset_x',
            default_value='-2.0',
            description='抓取点沿视觉/测量坐标系 x 轴的偏移（cm）',
        ),
        DeclareLaunchArgument(
            'grasp_offset_y',
            default_value='5.0',
            description='抓取点沿视觉/测量坐标系 y 轴的偏移（cm）',
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
