cmake_minimum_required(VERSION 2.8.3)
project(er_wheels)

find_package(catkin REQUIRED COMPONENTS
  tf2
  tf2_geometry_msgs
  phidgets
  geometry_msgs
  std_msgs
  roscpp
)

add_compile_options(-std=c++11)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs phidgets tf2
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(er_cartesian_motor_controller_node src/er_cartesian_motor_controller_node.cpp)
add_executable(er_motor_controller_node src/er_motor_controller_node.cpp)
add_executable(er_wheel_odometry_node src/er_wheel_odometry_node.cpp)

add_dependencies(er_cartesian_motor_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(er_motor_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(er_wheel_odometry_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(er_cartesian_motor_controller_node ${catkin_LIBRARIES})
target_link_libraries(er_motor_controller_node ${catkin_LIBRARIES})
target_link_libraries(er_wheel_odometry_node ${catkin_LIBRARIES})
