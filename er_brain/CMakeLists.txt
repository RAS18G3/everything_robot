cmake_minimum_required(VERSION 2.8.3)
project(er_brain)

find_package(catkin REQUIRED COMPONENTS
)

add_compile_options(-std=c++11)

catkin_package(
  INCLUDE_DIRS #include
  CATKIN_DEPENDS
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)
