cmake_minimum_required(VERSION 2.8.3)
project(caros_common_msgs)

########################################################################
#### Local Variables
########################################################################

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

########################################################################
#### Declare ROS Messages, Services and Actions
########################################################################
add_message_files(
  FILES
  q.msg
  caros_node_state.msg
  wrench_data.msg
  rw_state_data.msg
  rw_state.msg
)

add_service_files(
  FILES
  start.srv
  stop.srv
  pause.srv
  config_bool.srv
  config_int.srv
  empty_srv.srv
  get_rw_state.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

########################################################################
#### Catkin Specific Configuration
########################################################################
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
