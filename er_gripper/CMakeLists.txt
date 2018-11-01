cmake_minimum_required(VERSION 2.8.3)
project(er_gripper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

add_compile_options(-std=c++11)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(er_gripper_node src/er_gripper_node.cpp)

add_dependencies(er_gripper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(er_gripper_node ${catkin_LIBRARIES})
