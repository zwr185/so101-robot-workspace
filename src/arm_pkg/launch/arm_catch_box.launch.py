"""Launch camera, box detection, coordinate conversion and IK nodes."""

from glob import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    Shutdown,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_video_device(context):
    """Return the requested device or auto-detect a stable camera path."""
    requested = LaunchConfiguration('video_device').perform(context).strip()
    if requested:
        return requested

    stable_paths = sorted(glob('/dev/v4l/by-id/*-video-index0'))
    if stable_paths:
        # usb_cam 无法处理 by-id 软链接，这里解析成真实的 /dev/videoN
        return os.path.realpath(stable_paths[0])

    devices = sorted(glob('/dev/video*'))
    return devices[0] if devices else '/dev/video0'


def _launch_nodes(context):
    """Resolve the camera device and build all pipeline nodes."""
    vision_share = get_package_share_directory('vision_pkg')
    cam_config = os.path.join(vision_share, 'config', 'cam.yaml')
    detect_config = os.path.join(vision_share, 'config', 'box_detect.yaml')
    tf_config = os.path.join(vision_share, 'config', 'box_tf.yaml')

    video_device = _resolve_video_device(context)
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

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[cam_config, {'video_device': video_device}],
    )

    joint_node = Node(
        package='arm_pkg',
        executable='arm_catch_joint_node',
        name='arm_catch_joint_node',
        output='screen',
        parameters=[{'calibration_file': calibration_file}],
    )

    detect_node = Node(
        package='vision_pkg',
        executable='box_detect_node',
        name='box_detect_node',
        output='screen',
        parameters=[detect_config],
    )

    tf_node = Node(
        package='vision_pkg',
        executable='box_tf_node',
        name='box_tf_node',
        output='screen',
        parameters=[tf_config],
    )

    catch_node = Node(
        package='arm_pkg',
        executable='arm_catch_box_node',
        name='arm_catch_box_node',
        output='screen',
        parameters=[{
            'calibration_file': calibration_file,
            'input_yaw_offset': input_yaw_offset,
            'grasp_offset_x': grasp_offset_x,
            'grasp_offset_y': grasp_offset_y,
        }],
        on_exit=[Shutdown()],
    )

    return [
        LogInfo(msg=f'使用相机设备: {video_device}'),
        LogInfo(msg=f'视觉坐标→底座坐标 yaw 旋转: {input_yaw_offset} 度'),
        LogInfo(msg='按键提示: s=停止抓取, g=重新抓取, q=退出 launch'),
        usb_cam_node,
        joint_node,
        detect_node,
        tf_node,
        catch_node,
    ]


def generate_launch_description():
    """Start usb_cam, the arm driver and the box catching pipeline."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value='',
            description='相机设备路径，留空则自动检测 /dev/v4l/by-id 下的稳定链接',
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
