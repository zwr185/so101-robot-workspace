# 真机抓取演示启动文件（全位姿 IK 版）
#
# 功能：
#   在 demo.launch.py 的基础上额外启动 grasp_realarm_gui.py，
#   通过图形界面输入木块坐标和 X/Y/Z 抓取偏移，执行 8 步抓取/放置流程。
# 组成：
#   demo.launch.py       : 真机 ros2_control + move_group + RViz
#   grasp_realarm_gui.py : PyQt5 抓取界面（固定姿态 IK + 关节目标）
# 启动参数：
#   use_rviz : 是否启动 RViz（默认 true）

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("arm_moveit_config")

    use_rviz = LaunchConfiguration("use_rviz")

    # 真机版 demo：robot_state_publisher + move_group + rviz
    # + ros2_control_node(arm_hardware) + 三个 controller spawner
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "demo.launch.py")
        ),
        launch_arguments={
            "use_fake_hardware": "false",
            "use_rviz": use_rviz,
        }.items(),
    )

    grasp_gui = Node(
        package="arm_moveit_config",
        executable="grasp_realarm_gui.py",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="true: 启动 RViz（MoveIt MotionPlanning 面板）",
        )
    )
    ld.add_action(demo)
    ld.add_action(grasp_gui)
    return ld
