# 真机抓取演示启动文件（wrist_roll 固定版）
#
# 功能：
#   与 grasp_realarm.launch.py 相同，只是把抓取界面换成
#   1_grasp_realarm_gui.py：第 3 步固定 wrist_roll=0，
#   用数值位置 IK 求前 4 个关节，让 gripper_frame_link 到达 GUI 给定位置，
#   再由 MoveIt 规划并执行关节轨迹。
# 组成：
#   demo.launch.py         : 真机 ros2_control + move_group + RViz
#   1_grasp_realarm_gui.py : PyQt5 抓取界面（位置 IK 版）
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
        executable="1_grasp_realarm_gui.py",
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
