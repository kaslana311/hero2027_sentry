cmake_minimum_required(VERSION 3.8)
project(towords_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  
add_compile_options(-O3)
set(CMAKE_BUILD_TYPE "Release")
# find dependencies
add_compile_definitions(DEBUG)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yaml-cpp REQUIRED)
include_directories(include/ ${YAML_CPP_INCLUDE_DIRS})
add_library(towords_planner SHARED src/towords_planner.cpp src/color_map_model.cpp src/theta_star.cpp)
ament_target_dependencies(towords_planner rclcpp nav2_common nav2_core nav2_costmap_2d nav2_msgs pluginlib rclcpp_action OpenCV)

target_link_libraries(towords_planner
    ${YAML_CPP_LIBRARIES}
)
install(TARGETS towords_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )
  install(DIRECTORY include/
  DESTINATION include/
  )
install (TARGETS towords_planner
  DESTINATION lib/${PROJECT_NAME}
  )
pluginlib_export_plugin_description_file(nav2_core towords_planner.xml)
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
