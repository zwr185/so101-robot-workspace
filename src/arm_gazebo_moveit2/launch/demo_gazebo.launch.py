"""MoveIt2 + Gazebo 联合启动。

在 gazebo.launch.py 的基础上追加：
  1. move_group：加载 URDF/SRDF、KDL 运动学、OMPL 规划管线，
     提供 /move_action、/compute_ik、/plan_kinematic_path 等接口
  2. apply_planning_scene：把木板、障碍墙写入 Planning Scene
  3. rviz2：加载 config/moveit.rviz，提供 MotionPlanning 面板

参数：
  headless              true 时只启动 gz server
  use_rviz              false 时不启动 RViz
  include_target_block  false 时不把待夹取方块加入 Planning Scene（抓取用）

注意：这里不启动 ros2_control_node，controller_manager 在 gz 进程内。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = get_package_share_directory("arm_gazebo_moveit2")

    # 与 Gazebo 使用同一份机器人描述（含 world 固定关节），
    # 只是这里不需要颜色覆盖，规划结果不受影响。
    moveit_config = (
        MoveItConfigsBuilder(
            "so101_new_follower", package_name="arm_gazebo_moveit2"
        )
        .robot_description(file_path="urdf/so101_new_follower.gazebo.xacro")
        .to_moveit_configs()
    )

    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    include_target_block = LaunchConfiguration("include_target_block")

    # 第二步的仿真 + ros2_control，不重复启动 ros2_control_node
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"headless": headless}.items(),
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
            },
        ],
    )

    # 把木板作为碰撞体加入 Planning Scene，让 MoveIt 能看到世界里的木板
    apply_scene_objects = Node(
        package="arm_gazebo_moveit2",
        executable="apply_planning_scene.py",
        output="screen",
        parameters=[
            {
                "include_target_block": ParameterValue(
                    include_target_block, value_type=bool
                )
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", os.path.join(pkg_share, "config", "moveit.rviz")],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        condition=IfCondition(use_rviz),
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
            description="true: 启动 RViz（MoveIt MotionPlanning 面板）",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "include_target_block",
            default_value="true",
            description="false: 不把待夹取方块加入 Planning Scene（抓取演示用）",
        )
    )
    ld.add_action(gazebo)
    ld.add_action(move_group)
    ld.add_action(apply_scene_objects)
    ld.add_action(rviz)
    return ld
