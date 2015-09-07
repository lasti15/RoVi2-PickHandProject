cmake_minimum_required(VERSION 2.8.3)
project(caros_common_robwork_msgs)

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
  RwStateData.msg
  RwState.msg
)

add_service_files(
  FILES
  GetRwState.srv
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
