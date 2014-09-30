cmake_minimum_required(VERSION 2.8.3)
project(caros_grasp_selection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs geometry_msgs sensor_msgs caros_common)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  GraspTarget.msg
)

add_service_files(
  FILES
  SelectGrasps.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs roscpp caros_common
)

########################################################################
#### Extra temporary - can contain information that can be useful for setting up this CMakeLists.txt
########################################################################
########################################################################
#### RobWork and RobWorkSim
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}/cmake")
MESSAGE("${RW_ROOT}")
#Include default settings for constructing a robwork dependent project
SET(RobWork_DIR "${RW_ROOT}/cmake")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")

include_directories(include ${ROBWORK_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(grasp_selection src/grasp_selection.cpp src/GraspDBRepository.cpp)
target_link_libraries(grasp_selection ${conv_library} ${catkin_LIBRARIES} ${ROBWORKSIM_LIBRARIES})

add_executable(robworkcell src/robworkcell.cpp)
target_link_libraries(robworkcell ${conv_library} ${catkin_LIBRARIES} ${ROBWORKSIM_LIBRARIES})

#add_executable(virtual_kin_platform src/VirtualKinPlatformMain.cpp src/VirtualKinematicPlatform.cpp)
#target_link_libraries(virtual_kin_platform ${conv_library} ${catkin_LIBRARIES} ${ROBWORK_LIBRARIES})
