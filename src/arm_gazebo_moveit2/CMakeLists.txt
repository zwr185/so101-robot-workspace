cmake_minimum_required(VERSION 3.22)
project(arm_gazebo_moveit2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config urdf worlds
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/apply_planning_scene.py
  scripts/grasp_demo.py
  scripts/grasp_demo_gui.py
  DESTINATION lib/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
