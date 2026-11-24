cmake_minimum_required(VERSION 3.8)
project(arm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


# 把模型文件安装到 install/share/arm_description/
install(DIRECTORY urdf meshes rviz launch
  DESTINATION share/${PROJECT_NAME}
)

# 自定义 joint_state_publisher GUI
install(PROGRAMS scripts/joint_state_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_state_gui
)

# URDF 位置反解服务
install(PROGRAMS scripts/ik_solver_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ik_solver_node
)

# GUI 和 IK 节点共用的运动学模块
install(FILES scripts/arm_kinematics.py
  DESTINATION lib/${PROJECT_NAME}
)

# GUI -> 真机驱动的遥操作转发节点
install(PROGRAMS scripts/simtoreal_controll_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME simtoreal_controll_node
)

# 真机驱动由 remote_control_pkg/follower_arm_joint 提供

ament_package()
