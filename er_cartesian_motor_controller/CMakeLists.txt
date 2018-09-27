cmake_minimum_required(VERSION 2.8.3)
project(er_cartesian_motor_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  roscpp
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(cartesian_motor_controller_node src/cartesian_motor_controller_node.cpp)
target_link_libraries(cartesian_motor_controller_node ${catkin_LIBRARIES})
