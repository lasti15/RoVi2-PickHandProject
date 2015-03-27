cmake_minimum_required(VERSION 2.8.3)
project(caros_sensor_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs caros_common_msgs)

################################################
## Declare ROS messages, services and actions ##
################################################
#add_service_files(
#  FILES
#)

add_message_files(
  FILES
  pose_sensor_state.msg
  tactile_array_data.msg
  tactile_array_state.msg
  button_sensor_state.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  caros_common_msgs
)
###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs caros_common_msgs
)
