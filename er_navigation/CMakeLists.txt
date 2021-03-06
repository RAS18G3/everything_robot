cmake_minimum_required(VERSION 2.8.3)
project(er_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  tf
  tf2
  geometry_msgs
  er_planning
  pcl_ros
  actionlib
  rosbag
  message_generation
)

add_service_files(
  FILES
  MapLoadSave.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # and/or other packages containing depended messages
)

add_compile_options(-std=c++11)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp nav_msgs tf tf2 geometry_msgs er_planning pcl_ros actionlib rosbag message_runtime
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(er_slam_node src/er_slam_node.cpp src/map_reader.cpp src/occupancy_grid_utils.cpp)
add_executable(look_forward_drive src/look_forward_drive.cpp)
add_executable(er_explore_node src/er_explore_node.cpp)

add_dependencies(er_slam_node er_navigation_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(look_forward_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(er_explore_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(er_slam_node ${catkin_LIBRARIES})
target_link_libraries(look_forward_drive ${catkin_LIBRARIES})
target_link_libraries(er_explore_node ${catkin_LIBRARIES})
