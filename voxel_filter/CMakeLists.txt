cmake_minimum_required(VERSION 3.8)
project(voxel_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(voxel_filter_node src/voxel_filter.cpp)

ament_target_dependencies(voxel_filter_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(voxel_filter_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  voxel_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
