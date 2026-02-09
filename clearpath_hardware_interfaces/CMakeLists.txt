cmake_minimum_required(VERSION 3.5)
project(clearpath_hardware_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(clearpath_platform_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


## COMPILE
# J100 Hardware
add_library(
  j100_hardware
  SHARED
  src/j100/hardware.cpp
  src/j100/hardware_interface.cpp
  src/diff_drive/hardware.cpp
  src/diff_drive/hardware_interface.cpp
)

target_include_directories(
  j100_hardware
  PRIVATE
  include
)

ament_target_dependencies(
  j100_hardware
  clearpath_platform_msgs
  hardware_interface
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface src/j100/hardware.xml)

# INSTALL
install(
  TARGETS j100_hardware
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/battery_state/battery_state_estimator
  ${PROJECT_NAME}/battery_state/battery_state_control
  DESTINATION lib/${PROJECT_NAME}
)

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  j100_hardware
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()
