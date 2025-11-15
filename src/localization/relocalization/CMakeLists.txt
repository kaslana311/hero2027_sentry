cmake_minimum_required(VERSION 3.5)
project(relocalization)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(small_gicp_INCLUDE_DIRS /usr/local/include/small_gicp/registration)
set(small_gicp_LIBRARIES /usr/local/lib/libsmall_gicp.so)

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(small_gicp REQUIRED)
find_package(pcl_conversions REQUIRED)
include_directories(include ${small_gicp_INCLUDE_DIRS})
# 添加可执行文件
add_executable(relocalization_node src/relocalization_node.cpp)

ament_target_dependencies(relocalization_node rclcpp geometry_msgs nav_msgs tf2 tf2_ros Eigen3 sensor_msgs pcl_conversions OpenMP tf2_geometry_msgs)

# 链接PCL
target_link_libraries(relocalization_node ${PCL_LIBRARIES} ${small_gicp_LIBRARIES} OpenMP::OpenMP_CXX)

# 安装目标
install(TARGETS relocalization_node
DESTINATION lib/${PROJECT_NAME})

# 安装launch文件和配置文件
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()