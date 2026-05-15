cmake_minimum_required(VERSION 3.5)
project(hero_serial)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(serial_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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

include_directories(include
        /usr/include
        /usr/local/include
        /usr/include/boost
        /usr/local/include/boost
        )

add_library(drv_CRC src/drv_CRC.c)
target_include_directories(drv_CRC PUBLIC include)
add_library(drv_cmd_com src/drv_cmd_com.c)
target_include_directories(drv_cmd_com PUBLIC include)
target_link_libraries(drv_cmd_com drv_CRC)

add_executable(chassis_node src/chassis_node.cpp src/boost_asio.cpp)
target_link_libraries(chassis_node drv_cmd_com drv_CRC)

ament_target_dependencies(chassis_node rclcpp geometry_msgs serial_interfaces nav_msgs interfaces std_msgs)

target_link_libraries(chassis_node Threads::Threads)

install(TARGETS
  chassis_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config include
  DESTINATION share/${PROJECT_NAME})

  ament_package()