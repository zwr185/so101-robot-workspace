import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 从安装目录找到 URDF，用 xacro 解析成 robot_description
    urdf_file = os.path.join(
        get_package_share_directory('arm_description'),
        'urdf', 'arm.urdf.xacro')
    robot_description =ParameterValue(
         Command(['xacro ', urdf_file]),
          value_type=str
)
    return LaunchDescription([
        Node(                       # ① 发布 TF：读模型 + 关节角 → 算出各 link 坐标
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(                       # ② 弹出滑块面板，手动给每个关节角度（开发期用）
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(                       # ③ 可视化（自动加载预配置，视角对准机械臂）
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('arm_description'),
                'rviz', 'arm.rviz')],
        ),
    ])