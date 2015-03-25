cmake_minimum_required(VERSION 2.8.3)
project(caros_trakstar)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs caros_common caros_sensor nodelet)

########################################################################
#### RobWork and RobWorkHardware
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")


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
    INCLUDE_DIRS include
    # CATKIN_DEPENDS:
    # - rosconsole is not used in the header files, so should not be added to the list here.
    CATKIN_DEPENDS roscpp caros_common caros_sensor nodelet
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

########################################################################
#### Extra temporary - can contain information that can be useful for setting up this CMakeLists.txt
########################################################################

# TrakStar stuff (6D Pose sensors)
ADD_DEFINITIONS( -DLINUX )
INCLUDE_DIRECTORIES( /opt/3DGuidance.Rev.E.64/3DGuidanceAPI)
SET(TRAKSTAR_LIBRARIES ATC3DGlib64)
LINK_DIRECTORIES(/opt/3DGuidance.Rev.E.64/3DGuidanceAPI )

# mband: link_directories is required until RobWork issue ## is solved
######link_directories(${ROBWORK_LIBRARY_DIRS})

add_library(trakstar	
    src/Trakstar.cpp
    src/trakstar_node.cpp
)
TARGET_LINK_LIBRARIES(trakstar ${ROBWORK_LIBRARIES} ${TRAKSTAR_LIBRARIES} ${catkin_LIBRARIES})

add_executable(caros_trakstar src/main.cpp)
target_link_libraries(caros_trakstar trakstar ${ROBWORK_LIBRARIES} ${TRAKSTAR_LIBRARIES} ${catkin_LIBRARIES})

########################################################################
#### Testing
########################################################################
#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest_gtest(simple_demo test/simple_demo.test test/simple_demo.cpp)
#  target_link_libraries(simple_demo ${catkin_LIBRARIES})
#endif()
