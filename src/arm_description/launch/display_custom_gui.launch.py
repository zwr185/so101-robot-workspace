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
