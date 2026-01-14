cmake_minimum_required(VERSION 3.8)
project(scan_handle_tool)
set(CMAKE_BUILD_TYPE "Debug")

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)
add_executable(scan_handle_tool sc_h.cpp)
target_link_libraries(scan_handle_tool  ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}")
install(TARGETS
  scan_handle_tool
  DESTINATION bin
)

