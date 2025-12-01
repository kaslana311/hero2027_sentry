cmake_minimum_required(VERSION 3.10 )
project(pcd2ply)
find_package(PCL REQUIRED)
add_executable(${PROJECT_NAME} pcd2ply.cpp)
include_directories(${PCL_DIRECTORIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_DIRECTORIES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

