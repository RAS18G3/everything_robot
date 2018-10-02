cmake_minimum_required(VERSION 2.8.3)
project(er_launch)

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
)

include_directories(
)
