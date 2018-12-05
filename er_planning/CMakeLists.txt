cmake_minimum_required(VERSION 2.8.3)
project(er_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  tf
  tf2
  nav_msgs
  roscpp
  actionlib_msgs
)

add_action_files(
  DIRECTORY action
  FILES Path.action
  )

generate_messages(
  DEPENDENCIES actionlib_msgs nav_msgs
)

catkin_package(
 # INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp tf tf2 actionlib_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(er_pathfinder_node src/er_pathfinder_node.cpp src/rrt_utils.cpp)

add_dependencies(er_pathfinder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(er_pathfinder_node ${catkin_LIBRARIES})
