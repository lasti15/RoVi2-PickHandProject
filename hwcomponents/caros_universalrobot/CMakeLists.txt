cmake_minimum_required(VERSION 2.8.3)
project(caros_universalrobot)

########################################################################
#### Local Variables
########################################################################
set(library_name ${PROJECT_NAME})

########################################################################
#### Make use of c++11
########################################################################
# See: http://answers.ros.org/question/173947/export-gcc-flags-though-catkin-cfg_extras/
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs roscpp rosconsole caros_common caros_control caros_control_msgs)

################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(
 FILES
 URServiceServoQ.srv
 URServiceServoT.srv
 URServiceEmpty.srv
 URServiceForceModeStart.srv
 URServiceForceModeUpdate.srv
 URServiceForceModeStop.srv
)

generate_messages(
 DEPENDENCIES
 std_msgs
 caros_common_msgs
 geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs roscpp caros_common caros_control caros_control_msgs
  # FIXME: Should this package advertise that it depends on RobWork/RobWorkHardware (will other packages that are using this package / header files be required to have RobWork and/or RobWorkHardware?)
)

########################################################################
#### RobWork and RobWorkSim
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
set(RWHW_ROOT "$ENV{RWHW_ROOT}")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")
# The following CMAKE_MODULE_PATH configuration is required due to RobWorkHardware placing the things in build instead of cmake as RobWork is doing.
#set(CMAKE_MODULE_PATH ${RWHW_ROOT}/b ${CMAKE_MODULE_PATH})
# FIXME: Verify if replacing COMPONENTS with REQUIRED will make this node fail building properly?
# FIXME: How to properly ensure that this caros node/component will just not compile if the RobWork and RobWorkHardware packages are not found?
find_package(RobWorkHardware REQUIRED COMPONENTS universalrobots PATHS "${RWHW_ROOT}")
# FIXME: is 'netft' as a RobWorkHardware component required for this ROS node?

# FIXME: Verify if this if-thing can be removed and replaced with a REQUIRED instead of COMPONENTS in the find_package(RobWorkHardware) cmake function.
if (NOT ROBWORKHARDWARE_FOUND)

  message(WARNING "Skipping ${PROJECT_NAME}. Requires both universalrobots and netft components of RobWorkHardware!")

else()

include_directories(
  include 
  ${ROBWORK_INCLUDE_DIR}
  ${ROBWORKHARDWARE_INCLUDE_DIR}
#  ${COVIS_INCLUDE_DIR}
#  ${caros_control_INCLUDE_DIRS}
#  ${caros_common_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(ur_node src/ur_main.cpp src/UniversalRobots.cpp src/URServiceInterface.cpp)
target_link_libraries(ur_node
  ${ROBWORK_LIBRARIES}
  ${ROBWORKHARDWARE_LIBRARIES}
#  ${COVIS_LIBRARIES}
  ${caros_control_LIBRARIES}
  ${caros_common_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(ur_node
  caros_common
  caros_control
)

endif()
