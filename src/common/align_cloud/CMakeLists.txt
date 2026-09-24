cmake_minimum_required(VERSION 3.8)
project(align_cloud)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
message("Build type: ${CMAKE_BUILD_TYPE}")

# find dependencies
find_package(small_gicp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(align_cloud_node s.cpp)

target_link_libraries(align_cloud_node
  small_gicp::small_gicp
  yaml-cpp
  ${PCL_LIBRARIES}
  Eigen3::Eigen
) 

install(TARGETS align_cloud_node
  RUNTIME DESTINATION bin
)
