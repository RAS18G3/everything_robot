cmake_minimum_required(VERSION 2.8.3)
project(er_perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  tf2
#  opencv3
#  cv_bridge
  rospy
  message_generation
)

add_compile_options(-std=c++11)

add_message_files(
  FILES
  Object.msg
  ObjectList.msg
  ClassifiedBoundingBox.msg
  ClassifiedImage.msg
)

add_service_files(
  FILES
  RemoveObject.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs sensor_msgs pcl_ros tf2
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(er_pointcloud_to_2d_node src/er_pointcloud_to_2d_node.cpp)
add_executable(er_obstacle_detection_node src/er_obstacle_detection_node.cpp)
add_executable(er_object_filter_node src/er_object_filter_node.cpp)

add_dependencies(er_pointcloud_to_2d_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(er_obstacle_detection_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(er_object_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(er_pointcloud_to_2d_node ${catkin_LIBRARIES})
target_link_libraries(er_obstacle_detection_node ${catkin_LIBRARIES})
target_link_libraries(er_object_filter_node ${catkin_LIBRARIES})
