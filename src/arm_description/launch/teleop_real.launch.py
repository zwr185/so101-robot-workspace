"""
通过joint_state_gui.py的gui，控制rviz2中机器人模型运动，同时真机从动臂同步运动。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    start_driver = LaunchConfiguration("start_driver")
    port = LaunchConfiguration("port")
    wrist_roll_sign = LaunchConfiguration("wrist_roll_sign")

    urdf_file = os.path.join(
        get_package_share_directory("arm_description"),
        "urdf",
        "arm.urdf.xacro",
    )
    robot_description = ParameterValue(
        Command(["xacro ", urdf_file]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="是否启动 RViz2",
            ),
            DeclareLaunchArgument(
                "start_driver",
                default_value="true",
                description="是否启动真机从动臂驱动 follower_arm_joint（调试时可设为 false）",
            ),
            DeclareLaunchArgument(
                "port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-1a86_USB_Single_Serial_5B7B015685-if00"
                ),
                description="真机从动臂串口；如使用 /dev/ttyACM0 请显式传入",
            ),
            DeclareLaunchArgument(
                "wrist_roll_sign",
                default_value="-1.0",
                description="wrist_roll 方向修正；真机反向时改为 1.0",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="arm_description",
                executable="ik_solver_node",
                output="screen",
            ),
            Node(
                package="arm_description",
                executable="joint_state_gui",
                output="screen",
            ),
            Node(
                package="arm_description",
                executable="simtoreal_controll_node",
                output="screen",
                parameters=[
                    {
                        "wrist_roll_sign": ParameterValue(
                            wrist_roll_sign, value_type=float
                        )
                    }
                ],
            ),
            Node(
                package="remote_control_pkg",
                executable="follower_arm_joint",
                output="screen",
                parameters=[
                    {
                        "port": port,
                        "input_topic": "/cmd_angles",
                        "calibration_file": (
                            "/home/zwr185/dev_ws/Seeed_RoboController/json/"
                            "my_awesome_follower_arm.json"
                        ),
                    }
                ],
                condition=IfCondition(start_driver),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    os.path.join(
                        get_package_share_directory("arm_description"),
                        "rviz",
                        "arm.rviz",
                    ),
                ],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
