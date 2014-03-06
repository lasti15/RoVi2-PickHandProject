cmake_minimum_required(VERSION 2.8.3)
project(caros_sensor)

set(library_name "${PROJECT_NAME}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs caros_common)

################################################
## Declare ROS messages, services and actions ##
################################################
#add_service_files(
#  FILES
#)

add_message_files(
  FILES
  PoseSensorState.msg
  TactileArrayData.msg
  TactileArrayState.msg
  ButtonSensorState.msg
  TrakStarPose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  caros_common
)
###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${library_name}
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs caros_common
  DEPENDS RobWork
)

########################################################################
#### Extra temporary - can contain information that can be useful for setting up this CMakeLists.txt
########################################################################
########################################################################
#### RobWork and RobWorkSim
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
#message("RW_ROOT: ${RW_ROOT}")
#Include default settings for constructing a robwork dependent project
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")
#message("robwork libraries: ${ROBWORK_LIBRARIES}")
include_directories(include ${ROBWORK_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
#link_directories(${ROBWORK_LIBRARY_DIRS})
#[ mband: This is not recommended ] - link_directories( ${ROBWORK_LIBRARY_DIRS})

#message("rw inc dir" ${ROBWORK_INCLUDE_DIR})
#message("rw link dir" ${ROBWORK_LIBRARY_DIRS})
#message("rw libraries" ${ROBWORK_LIBRARIES})
#message("rw libs" ${ROBWORK_LIBS})

add_library(${library_name} 
	src/ButtonSensorSIProxy.cpp
	src/FTSensorServiceInterface.cpp
	src/FTSensorSIProxy.cpp
	src/PoseSensorServiceInterface.cpp
	src/PoseSensorSIProxy.cpp
)

target_link_libraries(${library_name}
  ${catkin_LIBRARIES}
  ${ROBWORK_LIBRARIES}
)

