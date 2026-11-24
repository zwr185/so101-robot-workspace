"""脚本化抓取演示（无 GUI）。

包含 demo_gazebo.launch.py（include_target_block:=false），
再启动 scripts/grasp_demo.py，按固定九步流程自动执行：
初始位姿 → 打开夹爪 → IK 到方块 → 闭合夹爪 → 回 up →
shoulder_pan 转负限位（放置）→ 释放 → 复位。

参数：
  headless  true 时只启动 gz server
  use_rviz  false 时不启动 RViz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("arm_gazebo_moveit2")

    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "demo_gazebo.launch.py")
        ),
        launch_arguments={
            "headless": headless,
            "use_rviz": use_rviz,
            # 抓取演示时目标方块不能作为碰撞体进入 Planning Scene，
            # 否则夹爪无法规划到方块所在位置。
            "include_target_block": "false",
        }.items(),
    )

    grasp_demo = Node(
        package="arm_gazebo_moveit2",
        executable="grasp_demo.py",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="true: 只启动 gz sim server，不打开 Gazebo GUI",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="true: 启动 RViz",
        )
    )
    ld.add_action(simulation)
    ld.add_action(grasp_demo)
    return ld
