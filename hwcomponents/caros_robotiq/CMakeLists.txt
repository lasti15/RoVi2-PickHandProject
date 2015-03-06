cmake_minimum_required(VERSION 2.8.3)
project(caros_robotiq)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs caros_common caros_control)

################################################
## Declare ROS messages, services and actions ##
################################################
#add_service_files(
#  FILES
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  caros_common
#)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs caros_common caros_control
)
include_directories(${catkin_INCLUDE_DIRS})

########################################################################
#### RobWork
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
#Include default settings for constructing a robwork dependent project
find_package(RobWork PATHS "${RW_ROOT};${RW_ROOT}/cmake")
include_directories(include ${ROBWORK_INCLUDE_DIR} ${caros_control_INCLUDE_DIRS})

########################################################################
#### RobWorkHardware
########################################################################
set(RWHW_ROOT "$ENV{RWHW_ROOT}")
find_package(RobWorkHardware COMPONENTS robotiq PATHS "${RWHW_ROOT}/cmake")
if ((NOT ROBWORK_FOUND) OR (NOT ROBWORKHARDWARE_FOUND))
  message(WARNING "Skipping ${PROJECT_NAME}. Requires RobWork and the robotiq component of RobWorkHardware!!!")
else()
  include_directories(include ${ROBWORKHARDWARE_INCLUDE_DIRS})

########################################################################
#### Executables
########################################################################
  add_executable(caros_robotiq3 src/main_robotiq3.cpp src/robotiq3_node.cpp)
  target_link_libraries(caros_robotiq3 ${conv_library}
    ${ROBWORK_LIBRARIES}
    ${ROBWORKHARDWARE_LIBRARIES}
    ${catkin_LIBRARIES}
  )
  add_dependencies(caros_robotiq3
    caros_common
    caros_control
  )
endif()
