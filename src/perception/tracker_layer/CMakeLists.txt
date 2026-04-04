cmake_minimum_required(VERSION 3.8)
project(tracker_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
message("Build type: ${CMAKE_BUILD_TYPE}")
# set (CMAKE_BUILD_TYPE "Release")
# add_compile_options(-O3)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) # 必须有这一行
find_package(PCL REQUIRED)
# find_package(backward_ros REQUIRED)
include_directories(
  include
)
add_library(tracker_layer SHARED
  src/tracker_layer.cpp
)
ament_target_dependencies(tracker_layer
  rclcpp
  sensor_msgs
  nav2_costmap_2d
  pluginlib
  tf2_ros
  tf2_eigen
  pcl_conversions
  PCL
  interfaces
  # backward_ros
  tf2_geometry_msgs
)
install(TARGETS tracker_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )
install (TARGETS tracker_layer
  DESTINATION lib/${PROJECT_NAME}
  )
pluginlib_export_plugin_description_file(nav2_costmap_2d tracker_layer.xml)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
