cmake_minimum_required(VERSION 3.8)
project(hero_sim_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(hero_goal_update_reset_bt_node SHARED src/reset_on_goal_update.cpp)
target_compile_features(hero_goal_update_reset_bt_node PRIVATE cxx_std_17)
target_compile_definitions(hero_goal_update_reset_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(hero_goal_update_reset_bt_node PRIVATE include)
ament_target_dependencies(hero_goal_update_reset_bt_node behaviortree_cpp_v3 geometry_msgs)
install(TARGETS hero_goal_update_reset_bt_node LIBRARY DESTINATION lib)
ament_export_libraries(hero_goal_update_reset_bt_node)
install(DIRECTORY include/ DESTINATION include)

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
  geometry_msgs rclcpp sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
)

install(TARGETS sim_ground_truth_bridge sim_point_lio_input_bridge sim_point_lio_evaluation
  sim_point_lio_cloud_bridge
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(sim_point_lio_navigation_bridge src/sim_point_lio_navigation_bridge.cpp)
ament_target_dependencies(sim_point_lio_navigation_bridge
  geometry_msgs nav_msgs rclcpp rcl_interfaces sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
)
install(TARGETS sim_point_lio_navigation_bridge DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY behavior_trees config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(nav2_msgs REQUIRED)
  find_package(rclcpp_action REQUIRED)
  add_executable(goal_update_reset_actions test/test_goal_update_reset_actions.cpp)
  target_compile_features(goal_update_reset_actions PRIVATE cxx_std_17)
  ament_target_dependencies(goal_update_reset_actions
    behaviortree_cpp_v3 geometry_msgs nav_msgs nav2_msgs rclcpp rclcpp_action)
  add_executable(goal_update_reset_control_flow test/test_goal_update_reset_control_flow.cpp)
  target_compile_features(goal_update_reset_control_flow PRIVATE cxx_std_17)
  ament_target_dependencies(goal_update_reset_control_flow behaviortree_cpp_v3 geometry_msgs nav_msgs)
  ament_add_pytest_test(goal_update_reset test/test_goal_update_reset.py
    TIMEOUT 90
    ENV "GOAL_RESET_TEST_EXECUTABLE=$<TARGET_FILE:goal_update_reset_control_flow>"
        "GOAL_RESET_PLUGIN=$<TARGET_FILE:hero_goal_update_reset_bt_node>"
        "GOAL_RESET_ACTIONS_EXECUTABLE=$<TARGET_FILE:goal_update_reset_actions>"
  )
  ament_add_pytest_test(cloud_registration test/test_cloud_registration.py
    TIMEOUT 120
    ENV "BRIDGE_EXECUTABLE=$<TARGET_FILE:sim_ground_truth_bridge>"
        "ROS_LOCALHOST_ONLY=1" "ROS_DOMAIN_ID=173"
  )
  ament_add_pytest_test(recovery_contract test/test_recovery_contract.py TIMEOUT 30)
  ament_add_pytest_test(costmap_modes test/test_costmap_modes.py TIMEOUT 30)
  ament_add_pytest_test(lidar_mount_contract test/test_lidar_mount_contract.py TIMEOUT 30)
  ament_add_pytest_test(split_mode test/test_split_mode.py TIMEOUT 30)
  ament_add_pytest_test(start_connector test/test_start_connector.py TIMEOUT 30)
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
  ament_add_pytest_test(point_lio_tf_chain test/test_point_lio_tf_chain.py
    TIMEOUT 150
    ENV "POINT_LIO_NAV_EXECUTABLE=$<TARGET_FILE:sim_point_lio_navigation_bridge>"
        "POINT_LIO_CLOUD_EXECUTABLE=$<TARGET_FILE:sim_point_lio_cloud_bridge>"
        "EVALUATION_EXECUTABLE=$<TARGET_FILE:sim_point_lio_evaluation>"
        "ROS_LOCALHOST_ONLY=1" "ROS_DOMAIN_ID=184"
  )
endif()

ament_package()
