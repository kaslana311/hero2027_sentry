cmake_minimum_required(VERSION 3.8)
project(cloud_analys)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_BUILD_TYPE "RELEASE")
add_compile_options(-O3)
# set (CMAKE_BUILD_TYPE "Debug")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
include_directories(include ${YAML_CPP_INCLUDE_DIRS})
add_executable(cloud_analys_node src/s.cpp)
ament_target_dependencies(cloud_analys_node rclcpp PCL tf2_ros nav_msgs sensor_msgs pcl_conversions pcl_ros OpenCV )
target_link_libraries(cloud_analys_node  ${YAML_CPP_LIBRARIES})
install(TARGETS
  cloud_analys_node
  DESTINATION lib/${PROJECT_NAME}
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
