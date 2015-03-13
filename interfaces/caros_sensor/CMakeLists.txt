cmake_minimum_required(VERSION 2.8.3)
project(caros_sensor)

########################################################################
#### Local Variables
########################################################################
set(library_name "${PROJECT_NAME}")

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs caros_common_msgs caros_common caros_sensor_msgs)

########################################################################
#### RobWork
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
#Include default settings for constructing a robwork dependent project
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")

################################################
## Declare ROS messages, services and actions ##
################################################
#add_service_files(
#  FILES
#)

# add_message_files(
#   FILES
# )

# generate_messages(
#   DEPENDENCIES
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${library_name}
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs caros_common_msgs caros_common caros_sensor_msgs
  DEPENDS RobWork
)

########################################################################
#### Build
########################################################################
include_directories(
  include
  ${ROBWORK_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${library_name} 
  src/pose_sensor_service_interface.cpp
  src/pose_sensor_si_proxy.cpp
  src/button_sensor_service_interface.cpp
  src/button_sensor_si_proxy.cpp
  
  src/FTSensorServiceInterface.cpp
  src/FTSensorSIProxy.cpp
  
)

add_dependencies(${library_name}
  caros_sensor_msgs_generate_messages_cpp
  caros_common
)

target_link_libraries(${library_name}
  ${catkin_LIBRARIES}
  ${ROBWORK_LIBRARIES}
)
