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

########################################################################
#### Extra temporary - can contain information that can be useful for setting up this CMakeLists.txt
########################################################################

########################################################################
#### RobWork
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")

SET(CMAKE_BUILD_TYPE Release)

#Include default settings for constructing a robwork dependent project
find_package(RobWork REQUIRED PATHS "${RW_ROOT};${RW_ROOT}/cmake")
include_directories(include ${ROBWORK_INCLUDE_DIR} ${caros_control_INCLUDE_DIRS})


########################################################################
#### RobWorkHardware
########################################################################

set(RWHW_ROOT "$ENV{RWHW_ROOT}")
find_package(RobWorkHardware REQUIRED COMPONENTS robotiq PATHS "${RWHW_ROOT}/cmake")
if (NOT ROBWORKHARDWARE_FOUND)
  message(WARNING "Skipping ${PROJECT_NAME}. Requires the robotiq component of RobWorkHardware!!!")
else()
include_directories(include ${ROBWORKHARDWARE_INCLUDE_DIRS})
link_directories(${ROBWORKHARDWARE_LIBRARY_DIRS})

########################################################################
#### Executables
########################################################################
add_executable(robotiq3 src/mainR3.cpp src/Robotiq3Node.cpp)
target_link_libraries(robotiq3 ${conv_library} 
  ${ROBWORK_LIBRARIES} 
  ${ROBWORKHARDWARE_LIBRARIES} 
  ${caros_common_LIBRARIES} 
  ${caros_control_LIBRARIES} 
  ${catkin_LIBRARIES}
)
add_dependencies(robotiq3
  caros_common
  caros_control
)

endif()
