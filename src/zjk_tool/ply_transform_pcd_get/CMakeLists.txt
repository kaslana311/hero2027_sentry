cmake_minimum_required(VERSION 3.10 )
project(ply_handle)
find_package(PCL REQUIRED)
add_executable(${PROJECT_NAME} handle.cpp)
include_directories(${PCL_DIRECTORIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_DIRECTORIES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

