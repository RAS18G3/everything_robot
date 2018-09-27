cmake_minimum_required(VERSION 2.8.3)
project(er_motor_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  phidgets
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(motor_controller_node src/motor_controller_node.cpp)
target_link_libraries(motor_controller_node ${catkin_LIBRARIES})
