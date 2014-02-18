cmake_minimum_required(VERSION 2.8.3)
project(caros_msgs)

########################################################################
#### Local variables
########################################################################
set(conv_library "${PROJECT_NAME}_conv")

########################################################################
#### Catkin packages
########################################################################
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

########################################################################
#### RobWork
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")
include_directories(include ${ROBWORK_INCLUDE_DIR})

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  Q.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
# Workaround for EXPORTED_TARGETS until catkin 0.5.81+ is used (ros hydro)
set(${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_generate_messages ${PROJECT_NAME}_generate_messages_cpp)
# End of workaround
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${conv_library}
  CATKIN_DEPENDS message_runtime std_msgs
  DEPENDS RobWork
# [ This will only work with catkin 0.5.81+ ]  EXPORTED_TARGETS ${PROJECT_NAME}_generate_messages ${PROJECT_NAME}_generate_messages_cpp
)

########################################################################
#### caros
########################################################################
add_library(${conv_library} src/caros.cpp)
add_dependencies(${conv_library} ${PROJECT_NAME}_generate_messages_cpp)