cmake_minimum_required(VERSION 3.8)
project(Bstyle_smoother)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlopt/install/lib/cmake/nlopt")
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
set(NLOPT_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlopt/install/lib")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3)
# 1. 允许在安装后保留 RPATH
set(CMAKE_INSTALL_RPATH "${NLOPT_LIB_DIR}")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(NLopt REQUIRED)
find_package(OpenCV REQUIRED)
message(WARNING "NLopt_INCLUDE_DIRS: ${NLopt_INCLUDE_DIRS}")
set (INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlopt/install/include")
# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${INCLUDE_DIR}
)

# Library
add_library(${PROJECT_NAME} SHARED
  src/bspline_optimizer.cpp
  src/bspline_smoother.cpp
  src/non_uniform_bspline.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  pluginlib
  tf2_ros
  OpenCV
)
set(LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlopt/install/lib")
target_link_libraries(${PROJECT_NAME}
  NLopt::nlopt
)

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Export plugin
pluginlib_export_plugin_description_file(nav2_core plugins.xml)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  pluginlib
  Eigen3
  tf2_ros
)


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
