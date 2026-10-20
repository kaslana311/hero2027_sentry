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
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

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

add_executable(sim_point_lio_evaluation src/sim_point_lio_evaluation.cpp)
ament_target_dependencies(sim_point_lio_evaluation
  geometry_msgs nav_msgs rclcpp std_msgs tf2 tf2_geometry_msgs tf2_ros visualization_msgs
)

add_executable(sim_point_lio_cloud_bridge src/sim_point_lio_cloud_bridge.cpp)
ament_target_dependencies(sim_point_lio_cloud_bridge
  geometry_msgs rclcpp sensor_msgs std_msgs tf2 tf2_geometry_msgs
)

install(TARGETS sim_ground_truth_bridge sim_point_lio_input_bridge sim_point_lio_evaluation
  sim_point_lio_cloud_bridge
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(sim_point_lio_navigation_bridge src/sim_point_lio_navigation_bridge.cpp)
ament_target_dependencies(sim_point_lio_navigation_bridge
  geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
)
install(TARGETS sim_point_lio_navigation_bridge DESTINATION lib/${PROJECT_NAME})

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
  ament_add_pytest_test(costmap_modes test/test_costmap_modes.py TIMEOUT 30)
  ament_add_pytest_test(split_mode test/test_split_mode.py TIMEOUT 30)
  ament_add_pytest_test(split_planner_runtime test/test_split_planner_runtime.py
    TIMEOUT 120 ENV "ROS_LOCALHOST_ONLY=1" "ROS_DOMAIN_ID=196"
  )
  ament_add_pytest_test(high_column_filter test/test_high_column_filter.py
    TIMEOUT 120
    ENV "ROS_LOCALHOST_ONLY=1" "ROS_DOMAIN_ID=187"
  )
  ament_add_pytest_test(dog_xy_resolution test/test_dog_xy_resolution.py
    TIMEOUT 120
    ENV "ROS_LOCALHOST_ONLY=1" "ROS_DOMAIN_ID=197"
  )
  ament_add_pytest_test(point_lio_input test/test_point_lio_input.py
    TIMEOUT 120
    ENV "POINT_LIO_INPUT_EXECUTABLE=$<TARGET_FILE:sim_point_lio_input_bridge>"
        "ROS_LOCALHOST_ONLY=1" "ROS_DOMAIN_ID=174"
  )
  ament_add_pytest_test(point_lio_evaluation test/test_point_lio_evaluation.py
    TIMEOUT 120
    ENV "EVALUATION_EXECUTABLE=$<TARGET_FILE:sim_point_lio_evaluation>"
        "ROS_LOCALHOST_ONLY=1" "ROS_DOMAIN_ID=176"
  )
  ament_add_pytest_test(point_lio_cloud test/test_point_lio_cloud.py
    TIMEOUT 120
    ENV "POINT_LIO_CLOUD_EXECUTABLE=$<TARGET_FILE:sim_point_lio_cloud_bridge>"
        "POINT_LIO_INPUT_EXECUTABLE=$<TARGET_FILE:sim_point_lio_input_bridge>"
        "EVALUATION_EXECUTABLE=$<TARGET_FILE:sim_point_lio_evaluation>"
        "ROS_LOCALHOST_ONLY=1" "ROS_DOMAIN_ID=180"
  )
  ament_add_pytest_test(point_lio_navigation test/test_point_lio_navigation.py
    TIMEOUT 120
    ENV "POINT_LIO_NAV_EXECUTABLE=$<TARGET_FILE:sim_point_lio_navigation_bridge>"
        "POINT_LIO_CLOUD_EXECUTABLE=$<TARGET_FILE:sim_point_lio_cloud_bridge>"
        "ROS_LOCALHOST_ONLY=1" "ROS_DOMAIN_ID=183"
  )
endif()

ament_package()
