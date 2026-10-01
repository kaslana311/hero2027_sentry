cmake_minimum_required(VERSION 3.8)
project(hero_sim_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(sim_ground_truth_bridge src/sim_ground_truth_bridge.cpp)
ament_target_dependencies(
  sim_ground_truth_bridge
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(sim_point_lio_input_bridge src/sim_point_lio_input_bridge.cpp)
ament_target_dependencies(sim_point_lio_input_bridge
  livox_ros_driver2 rclcpp sensor_msgs tf2
)

install(TARGETS sim_ground_truth_bridge sim_point_lio_input_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(cloud_registration test/test_cloud_registration.py
    TIMEOUT 120
    ENV "BRIDGE_EXECUTABLE=$<TARGET_FILE:sim_ground_truth_bridge>"
        "ROS_LOCALHOST_ONLY=1" "ROS_DOMAIN_ID=173"
  )
  ament_add_pytest_test(recovery_contract test/test_recovery_contract.py TIMEOUT 30)
  ament_add_pytest_test(point_lio_input test/test_point_lio_input.py
    TIMEOUT 120
    ENV "POINT_LIO_INPUT_EXECUTABLE=$<TARGET_FILE:sim_point_lio_input_bridge>"
        "ROS_LOCALHOST_ONLY=1" "ROS_DOMAIN_ID=174"
  )
endif()

ament_package()
