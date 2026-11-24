"""带 PyQt5 界面的抓取演示。

包含 demo_gazebo.launch.py（include_target_block:=false），
再启动 scripts/grasp_demo_gui.py：
  - 界面设置木块坐标、抓取点 Y/Z 偏移、放置位置、速度/加速度缩放
  - 复位 / 抓取 / 停止按钮，日志实时输出
  - 放置位置参数保存到 ~/.config/arm_gazebo_moveit2/grasp_gui.yaml

参数：
  headless  true 时只启动 gz server
  use_rviz  false 时不启动 RViz

GUI 需要显示器，SSH 无显示环境不可用。
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
            # 抓取时木块不进入 Planning Scene，避免夹爪无法规划到木块位置
            "include_target_block": "false",
        }.items(),
    )

    grasp_gui = Node(
        package="arm_gazebo_moveit2",
        executable="grasp_demo_gui.py",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="true: 只启动 gz sim server（GUI 需要显示器）",
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
    ld.add_action(grasp_gui)
    return ld
