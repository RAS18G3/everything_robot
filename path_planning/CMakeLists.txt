cmake_minimum_required(VERSION 2.8.3)
project(path_planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  tf
  nav_msgs
  roscpp
  rospy
)


catkin_package(
 # INCLUDE_DIRS include
#  LIBRARIES path_planning
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy tf
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(send_odom src/send_odom.cpp)
add_executable(drive_node src/drive_node.cpp)
add_executable(path_node src/path_node.cpp)

add_dependencies(send_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(drive_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(path_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(send_odom ${catkin_LIBRARIES})
target_link_libraries(drive_node ${catkin_LIBRARIES})
target_link_libraries(path_node ${catkin_LIBRARIES})


