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

## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/mobile_base_node.cpp)
add_executable(move_minibot_node src/move_minibot_node.cpp)

## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(move_minibot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(move_minibot_node ${catkin_LIBRARIES})
