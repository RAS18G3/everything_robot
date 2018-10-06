cmake_minimum_required(VERSION 2.8.3)
project(er_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
)

add_compile_options(-std=c++11)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp nav_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(er_slam_node src/er_slam_node.cpp src/map_reader.cpp)

add_dependencies(er_slam_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(er_slam_node ${catkin_LIBRARIES})
