# 真机 MoveIt2 基础启动文件
#
# 功能：
#   启动真机 ros2_control 控制链路和 MoveIt2 规划执行所需的全部节点，
#   是 arm_moveit_config 中其他真机 launch 的基础。
# 包含节点：
#   robot_state_publisher : 发布 URDF 和 TF
#   move_group            : MoveIt 规划、Planning Scene、轨迹执行
#   rviz2                 : MotionPlanning 面板（由 use_rviz 控制）
#   ros2_control_node     : controller_manager，加载 arm_hardware 硬件插件
#   spawner               : manipulator_controller / gripper_controller /
#                           joint_state_broadcaster
# 启动参数：
#   use_fake_hardware : false 用真机 St3215SystemHardware，true 用 mock_components
#   use_rviz          : 是否启动 RViz
#   db                : 是否启动 warehouse 数据库

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_rviz = LaunchConfiguration("use_rviz")
    db = LaunchConfiguration("db")

    moveit_config = (
        MoveItConfigsBuilder("so101_new_follower", package_name="arm_moveit_config")
        .robot_description(mappings={"use_fake_hardware": use_fake_hardware})
        .to_moveit_configs()
    )

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="true: use mock_components/GenericSystem instead of the real STS3215 hardware",
        )
    )
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="true"))
    ld.add_action(DeclareLaunchArgument("db", default_value="false"))

    # The robot description is the fake-hardware variant when use_fake_hardware:=true.
    # robot_state_publisher also publishes it on /robot_description, which is what
    # ros2_control_node below consumes.
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            respawn=True,
            output="screen",
            parameters=[
                moveit_config.robot_description,
                {"publish_frequency": 15.0},
            ],
        )
    )

    ld.add_action(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {
                    "publish_robot_description_semantic": True,
                    "allow_trajectory_execution": True,
                    "publish_planning_scene": True,
                    "publish_geometry_updates": True,
                    "publish_state_updates": True,
                    "publish_transforms_updates": True,
                    "monitor_dynamics": False,
                    # 真机串口延迟 + 舵机加速慢，放宽 MoveIt 的执行监控，
                    # 否则轨迹稍慢就会被取消并报 CONTROL_FAILED(-4)
                    "trajectory_execution.allowed_start_tolerance": 0.1,
                    "trajectory_execution.allowed_execution_duration_scaling": 3.0,
                    "trajectory_execution.allowed_goal_duration_margin": 2.0,
                },
            ],
            additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
        )
    )

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=[
                "-d",
                str(moveit_config.package_path / "config/moveit.rviz"),
            ],
            parameters=[
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
            condition=IfCondition(use_rviz),
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                str(moveit_config.package_path / "config/ros2_controllers.yaml")
            ],
            remappings=[
                ("/controller_manager/robot_description", "/robot_description")
            ],
        )
    )

    controller_names = (
        moveit_config.trajectory_execution.get(
            "moveit_simple_controller_manager", {}
        ).get("controller_names", [])
    )
    for controller in controller_names + ["joint_state_broadcaster"]:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(db),
        )
    )

    return ld
