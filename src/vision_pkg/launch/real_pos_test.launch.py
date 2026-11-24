"""Launch camera, detection and pixel-to-base conversion for accuracy tests."""

# 使用方法:
#   ros2 launch vision_pkg real_pos_test.launch.py
# 该 launch 会启动 usb_cam、box_detect_node、box_tf_node 和 rqt_topic，
# 用于观察像素坐标到机械臂底座厘米坐标的转换结果和标定精度。
#
# 操作步骤:
#   1. 在 rqt_topic 中勾选 /box_center 查看像素 (u, v)，勾选
#      /box_world_pos 查看转换后的底座坐标 (x, y, z)，单位 cm。
#   2. 把球放在相对底座已知的位置，比较 /box_world_pos 与实际坐标。
#   3. 偏差较大时，先运行 real_pos_locate.launch.py 重新标定，
#      再重新启动本 launch 验证。

from glob import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
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
    """Build the camera, detection and coordinate conversion nodes."""
    share_dir = get_package_share_directory('vision_pkg')
    cam_config = os.path.join(share_dir, 'config', 'cam.yaml')
    detect_config = os.path.join(share_dir, 'config', 'box_detect.yaml')
    tf_config = os.path.join(share_dir, 'config', 'box_tf.yaml')

    video_device = _resolve_video_device(context)
    homography_file = LaunchConfiguration('homography_file').perform(
        context
    ).strip()

    tf_parameters = [tf_config]
    if homography_file:
        tf_parameters.append({'homography_file': homography_file})

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[cam_config, {'video_device': video_device}],
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
        parameters=tf_parameters,
    )

    rqt_topic_node = Node(
        package='rqt_topic',
        executable='rqt_topic',
        name='rqt_topic',
        output='screen',
    )

    return [
        LogInfo(msg=f'使用相机设备: {video_device}'),
        LogInfo(
            msg=f'/box_world_pos 使用透视变换文件: '
            f'{homography_file or "box_tf.yaml 中的配置"}'
        ),
        LogInfo(
            msg='已打开 rqt_topic：勾选 /box_center（像素 u,v）和 '
            '/box_world_pos（cm）查看实时坐标'
        ),
        usb_cam_node,
        detect_node,
        tf_node,
        rqt_topic_node,
    ]


def generate_launch_description():
    """Start camera, detection and pixel-to-base conversion."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value='',
            description='相机设备路径，留空则自动检测 /dev/v4l/by-id 下的稳定链接',
        ),
        DeclareLaunchArgument(
            'homography_file',
            default_value=(
                '/home/zwr185/dev_ws/src/vision_pkg/json/'
                'box_tf_homography.json'
            ),
            description='像素→底座坐标的透视变换文件',
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
