"""Launch camera and detection, then open the pixel-to-base calibration tool."""

# 使用方法:
#   ros2 launch vision_pkg real_pos_locate.launch.py
# 该 launch 会启动 usb_cam、box_detect_node 和 rqt_topic；如果系统里有
# gnome-terminal，会自动打开一个新终端运行 box_tf_calibrate 标定程序，
# 否则会在日志里打印需要手动执行的标定命令。
#
# 标定步骤:
#   1. 在 rqt_topic 中勾选 /box_center，确认像素 u,v 持续刷新。
#   2. 把球放在相对机械臂底座已知的 (x, y) 位置，在标定终端输入
#      "x y" 并回车，程序会采集当前 /box_center 像素。
#   3. 更换位置重复采集，至少 4 个点，建议覆盖整个工作区域。
#   4. 输入 q 结束标定，结果默认保存到
#      /home/zwr185/dev_ws/src/vision_pkg/json/box_tf_homography.json。
#   5. 重新启动 box_tf_node（或 real_pos_test.launch.py）使新参数生效。

from glob import glob
import os
import shlex
import shutil

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
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
    """Start the vision pipeline and the interactive calibration tool."""
    share_dir = get_package_share_directory('vision_pkg')
    cam_config = os.path.join(share_dir, 'config', 'cam.yaml')
    detect_config = os.path.join(share_dir, 'config', 'box_detect.yaml')

    video_device = _resolve_video_device(context)
    output = LaunchConfiguration('output').perform(context).strip()
    if not output:
        output = (
            '/home/zwr185/dev_ws/src/vision_pkg/json/'
            'box_tf_homography.json'
        )
    pixel_topic = LaunchConfiguration('pixel_topic').perform(context).strip()
    if not pixel_topic:
        pixel_topic = '/box_center'

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

    rqt_topic_node = Node(
        package='rqt_topic',
        executable='rqt_topic',
        name='rqt_topic',
        output='screen',
    )

    install_dir = os.path.normpath(
        os.path.join(get_package_prefix('vision_pkg'), '..', '..')
    )
    setup_file = os.path.join(install_dir, 'setup.bash')
    calibrate_cmd = (
        f'source /opt/ros/jazzy/setup.bash && '
        f'source {shlex.quote(setup_file)} && '
        f'ros2 run vision_pkg box_tf_calibrate '
        f'--output {shlex.quote(output)} '
        f'--pixel-topic {shlex.quote(pixel_topic)}'
    )
    terminal_script = (
        calibrate_cmd
        + '; echo; echo "标定结束，关闭本窗口即可"; exec bash'
    )

    actions = [
        LogInfo(msg=f'使用相机设备: {video_device}'),
        LogInfo(msg=f'标定结果将保存到: {output}'),
        LogInfo(
            msg='已打开 rqt_topic：勾选 /box_center 可实时查看像素 u,v'
        ),
        usb_cam_node,
        detect_node,
        rqt_topic_node,
    ]

    if shutil.which('gnome-terminal'):
        actions.append(ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', terminal_script],
            output='screen',
        ))
        actions.append(LogInfo(
            msg='已打开标定终端：把球放在已知位置，输入相对底座的 x y（cm），'
            '完成全部点后输入 q 保存'
        ))
    else:
        actions.append(LogInfo(
            msg='未找到 gnome-terminal，请另开终端运行:\n' + calibrate_cmd
        ))

    return actions


def generate_launch_description():
    """Start the vision pipeline and calibration tool for base-frame points."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value='',
            description='相机设备路径，留空则自动检测 /dev/v4l/by-id 下的稳定链接',
        ),
        DeclareLaunchArgument(
            'output',
            default_value=(
                '/home/zwr185/dev_ws/src/vision_pkg/json/'
                'box_tf_homography.json'
            ),
            description='透视变换结果的保存路径',
        ),
        DeclareLaunchArgument(
            'pixel_topic',
            default_value='/box_center',
            description='检测节点发布的像素中心话题',
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
