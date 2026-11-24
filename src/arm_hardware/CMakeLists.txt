cmake_minimum_required(VERSION 3.8)
project(arm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(arm_hardware SHARED
  src/st3215_system_hardware.cpp
)
target_include_directories(arm_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(arm_hardware PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(arm_hardware rclcpp hardware_interface pluginlib)

# 导出插件描述，让 ros2_control 能加载 arm_hardware/St3215SystemHardware
pluginlib_export_plugin_description_file(hardware_interface arm_hardware_plugins.xml)

install(TARGETS arm_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_dependencies(rclcpp hardware_interface pluginlib)
ament_export_include_directories(include)
ament_export_libraries(arm_hardware)

ament_package()