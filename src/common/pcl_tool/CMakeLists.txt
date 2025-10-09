cmake_minimum_required(VERSION 3.5)
project(pcl_tool)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include 
${PCL_INCLUDE_DIRS})

# add_executable(read_pcd src/reading_PCD.cpp)
# add_executable(writing_pcd src/writing_PCD.cpp)
# add_executable(c_clouds src/concatenate_clouds.cpp)
# add_executable(passthrough src/passthrough_filter.cpp)
add_executable(voxel src/voxelgrid_filter.cpp)
# add_executable(remove src/statistical_outlier_removal.cpp)
# add_executable(icp src/icp.cpp)
add_executable(filter_cloud src/condition_filter.cpp)
add_executable(filter_z src/filter_z.cpp)





# ament_target_dependencies(read_pcd rclcpp)
# ament_target_dependencies(writing_pcd rclcpp)
# ament_target_dependencies(c_clouds rclcpp)
# ament_target_dependencies(passthrough rclcpp)
ament_target_dependencies(voxel rclcpp)
ament_target_dependencies(filter_z rclcpp)

# ament_target_dependencies(remove rclcpp)
# ament_target_dependencies(icp rclcpp)
# ament_target_dependencies(ndt rclcpp)
# ament_target_dependencies(con rclcpp)
ament_target_dependencies(filter_cloud rclcpp sensor_msgs nav_msgs tf2 tf2_ros) 




# target_link_libraries(read_pcd ${PCL_LIBRARIES})
# target_link_libraries(writing_pcd ${PCL_LIBRARIES})
# target_link_libraries(c_clouds ${PCL_LIBRARIES})
# target_link_libraries(passthrough ${PCL_LIBRARIES})
target_link_libraries(voxel ${PCL_LIBRARIES})
target_link_libraries(filter_z ${PCL_LIBRARIES})

# target_link_libraries(remove ${PCL_LIBRARIES})
# target_link_libraries(icp ${PCL_LIBRARIES})
# target_link_libraries(ndt ${PCL_LIBRARIES} Eigen3::Eigen)
target_link_libraries(filter_cloud ${PCL_LIBRARIES} Eigen3::Eigen)




# install(TARGETS read_pcd writing_pcd c_clouds passthrough voxel icp ndt filter_cloud 
#   DESTINATION lib/${PROJECT_NAME}
# )

install(TARGETS filter_cloud voxel filter_z
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
