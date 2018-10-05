cmake_minimum_required(VERSION 2.8.3)
project(er_wheels)

find_package(catkin REQUIRED COMPONENTS
  phidgets
  geometry_msgs
  std_msgs
  roscpp
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(er_cartesian_motor_controller_node src/er_cartesian_motor_controller_node.cpp)
add_executable(er_motor_controller_node src/er_motor_controller_node.cpp)

target_link_libraries(er_cartesian_motor_controller_node ${catkin_LIBRARIES})
target_link_libraries(er_motor_controller_node ${catkin_LIBRARIES})
