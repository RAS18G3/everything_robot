cmake_minimum_required(VERSION 2.8.3)
project(er_perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  tf2
)

add_compile_options(-std=c++11)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros tf2
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(er_pointcloud_to_2d_node src/er_pointcloud_to_2d_node.cpp)

add_dependencies(er_pointcloud_to_2d_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(er_pointcloud_to_2d_node ${catkin_LIBRARIES})
