cmake_minimum_required(VERSION 2.8.3)
project(caros_control)

########################################################################
#### Local Variables
########################################################################
set(library_name "${PROJECT_NAME}")

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs caros_common)

########################################################################
#### RobWork
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")

################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(
  FILES
   SerialDeviceForceControlStart.srv
   SerialDeviceForceControlStop.srv
   SerialDeviceForceControlUpdate.srv
   SerialDeviceMoveLin.srv
   SerialDeviceMoveLinFC.srv
   SerialDeviceMovePTP_T.srv
   SerialDeviceMovePTP.srv
   SerialDeviceMoveVelQ.srv
   SerialDeviceMoveVelT.srv
  GripperMoveQ.srv
  GripperGripQ.srv
  GripperSetForceQ.srv
  GripperSetVelocityQ.srv
  GripperStopMovement.srv
)

add_message_files(
  FILES
  RobotState.msg
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
#### Build
########################################################################
include_directories(
  include
  ${ROBWORK_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# Create one library for the whole package (don't split it into gripper, serialDevice, etc. - if this is wanted then this package should rather be split into gripper, serialDevice and similar packages) 
add_library(${library_name}
  src/GripperServiceInterface.cpp
)

add_dependencies(${library_name}
# catkin_EXPORTED_TARGETS should be used if this target depends on things (e.g. message or service headers) that are imported from other catkin packages. (see http://docs.ros.org/hydro/api/catkin/html/howto/building_msgs.html)
# OBS: But this will cause every target in a package to be required and thereby requiring that target to build code dependent on e.g. RobWork and CoViS.
# OBS2: What happens once a third package wants to use the second package? (does the third package need to also depend on the first package and making it a very bad design solution to have msg,srv,action in the same package as the interfaces/other API?) or is it not required for msg,srv,action as their dependencies should be resolved through the generate_messages(DEPENDENCIES...)?
#  ${catkin_EXPORTED_TARGETS}
##
# caros_common_EXPORTED_TARGETS can also be used
# So use specific dependencies instead
  caros_common_generate_messages_cpp
#####
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(${library_name}
  ${catkin_LIBRARIES}
  ${ROBWORK_LIBRARIES}
)
