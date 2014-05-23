cmake_minimum_required(VERSION 2.8.3)
project(caros_common)

########################################################################
#### Local Variables
########################################################################
set(library_name "${PROJECT_NAME}")

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS
  message_generation std_msgs
)

########################################################################
#### RobWork
########################################################################
# RobWork is not directly required, as it's not linked with, but remains here due to common.hpp including rw/math.hpp
set(RW_ROOT "$ENV{RW_ROOT}")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")

########################################################################
#### Declare ROS Messages, Services and Actions
########################################################################
add_message_files(
  FILES
  Q.msg
  CarosNodeState.msg
)

add_service_files(
  FILES
  Start.srv
  Stop.srv
  Pause.srv
  ConfigBool.srv
  ConfigInt.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

########################################################################
#### Catkin Specific Configuration
########################################################################
# Workaround for EXPORTED_TARGETS until catkin 0.5.81+ is used (ros hydro)
#set(${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_generate_messages ${PROJECT_NAME}_generate_messages_cpp)
# End of workaround
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${library_name}
  CATKIN_DEPENDS message_runtime std_msgs
  DEPENDS RobWork
# RobWork is required to use the convertion functions as common.hpp is including rw/math.hpp.
# [ This will only work with catkin 0.5.81+ ]  EXPORTED_TARGETS ${PROJECT_NAME}_generate_messages ${PROJECT_NAME}_generate_messages_cpp
)

include_directories(include ${ROBWORK_INCLUDE_DIRS})

# Create one library for the whole package (don't split it into gripper, serialDevice, etc. - if this is wanted then this package should rather be split into gripper, serialDevice and similar packages) 
add_library(${library_name}
  src/CarosNodeServiceInterface.cpp
)



