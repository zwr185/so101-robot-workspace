"""Gazebo 仿真 + ros2_control 底层启动。

启动内容：
  1. gz sim 加载 worlds/so101_empty.sdf（地面、木板、障碍墙、待夹取方块）
  2. robot_state_publisher 发布 /robot_description 与固定关节 TF
  3. ros_gz_sim create 把机器人实体生成到世界中
  4. ros_gz_bridge 桥接 /clock
  5. 模型生成后由 spawner 依次加载 joint_state_broadcaster、
     manipulator_controller、gripper_controller

参数：
  headless  true 时只启动 gz server，不打开 Gazebo GUI

机器人描述由 urdf/so101_new_follower.gazebo.xacro 生成，并按
config/gazebo_colors.yaml 覆盖 link 颜色。本文件不涉及 MoveIt。
"""

import os
import xml.etree.ElementTree as ET

import xacro
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 必须与 config/ros2_controllers.yaml 中声明的控制器一致
CONTROLLERS = [
    "joint_state_broadcaster",
    "manipulator_controller",
    "gripper_controller",
]

# Gazebo 里的 link 颜色由 config/gazebo_colors.yaml 配置。
# 注意 gz sim 会丢弃 URDF 中 <gazebo reference="link"> 下的 <material>，
# 只有 <visual> 内部的 <material><color> 才会转成 SDF 材质，
# 所以这里在生成 robot_description 时直接写进 <visual>。
def _apply_link_colors(urdf_xml, link_colors):
    root = ET.fromstring(urdf_xml)
    for link in root.findall("link"):
        name = link.get("name")
        rgba = link_colors.get(name)
        if rgba is None:
            continue
        rgba_str = " ".join(str(channel) for channel in rgba)
        for visual in link.findall("visual"):
            old = visual.find("material")
            if old is not None:
                visual.remove(old)
            material = ET.SubElement(visual, "material", {"name": f"gazebo_{name}"})
            ET.SubElement(material, "color", {"rgba": rgba_str})
    return ET.tostring(root, encoding="unicode")


def generate_launch_description():
    pkg_share = get_package_share_directory("arm_gazebo_moveit2")
    world_file = os.path.join(pkg_share, "worlds", "so101_empty.sdf")
    xacro_file = os.path.join(pkg_share, "urdf", "so101_new_follower.gazebo.xacro")
    colors_file = os.path.join(pkg_share, "config", "gazebo_colors.yaml")

    with open(colors_file, "r", encoding="utf-8") as f:
        link_colors = yaml.safe_load(f) or {}

    robot_description = _apply_link_colors(
        xacro.process_file(xacro_file).toxml(), link_colors
    )

    # Gazebo 用 GZ_SIM_RESOURCE_PATH 解析 URDF 里的 package://arm_description/...
    # get_package_prefix 返回 install/arm_description，其下的 share/ 目录正好是
    # 包名目录 arm_description 的父目录，mesh 才能被找到。
    gz_resource_paths = [os.path.join(get_package_prefix("arm_description"), "share")]
    if os.environ.get("GZ_SIM_RESOURCE_PATH"):
        gz_resource_paths.append(os.environ["GZ_SIM_RESOURCE_PATH"])
    gz_env = {"GZ_SIM_RESOURCE_PATH": os.pathsep.join(gz_resource_paths)}

    headless = LaunchConfiguration("headless")

    # headless:=true 时不启动 Gazebo GUI（SSH / 无显示环境用）
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v", "3", world_file],
        output="screen",
        additional_env=gz_env,
        on_exit=Shutdown(),
        condition=UnlessCondition(headless),
    )
    gz_sim_headless = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", "-v", "3", world_file],
        output="screen",
        additional_env=gz_env,
        on_exit=Shutdown(),
        condition=IfCondition(headless),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    # 等 Gazebo 世界加载完成（UserCommands 的 create 服务就绪）后再生成模型
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        parameters=[
            {
                "topic": "/robot_description",
                "name": "so101_new_follower",
                "allow_renaming": False,
                "z": 0.0,
            }
        ],
    )

    # gz -> ROS 的时钟桥；没有它 /joint_states 的时间戳会和 ROS 时间对不上
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # 控制器必须在模型生成之后才能加载（gz_ros2_control 的 controller_manager
    # 随着模型插入世界才会出现）
    controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name],
            output="screen",
        )
        for name in CONTROLLERS
    ]

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="true: 只启动 gz sim server，不打开 Gazebo GUI",
        )
    )
    ld.add_action(gz_sim)
    ld.add_action(gz_sim_headless)
    ld.add_action(robot_state_publisher)
    ld.add_action(clock_bridge)
    ld.add_action(TimerAction(period=5.0, actions=[spawn_entity]))
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=controller_spawners,
            )
        )
    )
    return ld
