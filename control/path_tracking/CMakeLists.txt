cmake_minimum_required(VERSION 3.5)
project(path_tracking)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
INCLUDE_DIRECTORIES(
${PROJECT_SOURCE_DIR}/src/include/spin_control
)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
include_directories(
include
${catkin_INCLUDE_DIRS}
/usr/local/include
  )
ament_package()
add_executable(spin_control src/spin_control.cpp)
ament_target_dependencies(spin_control rclcpp nav_msgs tf2_ros geometry_msgs tf2_geometry_msgs std_msgs tf2 serial_interfaces visualization_msgs)
install(TARGETS
spin_control
  DESTINATION lib/${PROJECT_NAME}
)
