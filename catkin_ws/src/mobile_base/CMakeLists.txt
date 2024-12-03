cmake_minimum_required(VERSION 3.0.2)
project(mobile_base)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  MoveMinibot.srv
  OdomSetPoint.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mobile_base
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs tf2_ros tf2_geometry_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mobile_base_node src/mobile_base_node.cpp)
add_dependencies(mobile_base_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mobile_base_node ${catkin_LIBRARIES})


add_executable(simple_move_node src/simple_move_node.cpp)
add_dependencies(simple_move_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_move_node ${catkin_LIBRARIES})