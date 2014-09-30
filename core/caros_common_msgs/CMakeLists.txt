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
)

########################################################################
#### Declare ROS Messages, Services and Actions
########################################################################
add_message_files(
  FILES
  Q.msg
  CarosNodeState.msg
  RWState.msg
)

add_service_files(
  FILES
  Start.srv
  Stop.srv
  Pause.srv
  ConfigBool.srv
  ConfigInt.srv
  GetRWState.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

########################################################################
#### Catkin Specific Configuration
########################################################################
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
