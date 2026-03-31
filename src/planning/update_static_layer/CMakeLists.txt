cmake_minimum_required(VERSION 3.8)
project(update_static_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set (CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3)
message("Build type: ${CMAKE_BUILD_TYPE}")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)
add_library(update_static_layer SHARED
  src/update_static_layer.cpp
)
target_link_libraries(update_static_layer
  ${OpenCV_LIBRARIES}
  yaml-cpp
)
ament_target_dependencies(update_static_layer
  rclcpp
  sensor_msgs
  nav2_costmap_2d
  pluginlib
  tf2_ros
  tf2_eigen
  pcl_conversions
  PCL
)
install(TARGETS update_static_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )
install (TARGETS update_static_layer
  DESTINATION lib/${PROJECT_NAME}
  )
pluginlib_export_plugin_description_file(nav2_costmap_2d update_static_layer.xml)
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
