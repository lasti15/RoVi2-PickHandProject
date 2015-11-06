cmake_minimum_required(VERSION 2.8.3)
project(caros_netft)



########################################################################
#### Make use of c++11
########################################################################
# See: http://answers.ros.org/question/173947/export-gcc-flags-though-catkin-cfg_extras/
# Should automatically use c++11 due to using caros_control
# Using hardcoded path-strings instead of a variable substitution for the use-c++11.cmake path-name, as catkin_lint doesn't properly do variable substitution (at least in this particular situation)
include("cmake/use-c++11.cmake")



########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS 
  caros_common 
  caros_common_msgs 
  caros_common_robwork 
  caros_sensor 
  caros_sensor_msgs  
  geometry_msgs     
  message_generation 
  roscpp 
  roslint
  std_msgs 
)


########################################################################
#### Roslint
########################################################################
set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()

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



########################################################################
#### Extra temporary - can contain information that can be useful for setting up this CMakeLists.txt
########################################################################
########################################################################
#### RobWork
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
#Include default settings for constructing a robwork dependent project
find_package(RobWork PATHS "${RW_ROOT};${RW_ROOT}/cmake")
include_directories(include ${ROBWORK_INCLUDE_DIR}  )




########################################################################
#### RobWorkHardware
########################################################################
set(RWHW_ROOT "$ENV{RWHW_ROOT}")
find_package(RobWorkHardware COMPONENTS netft PATHS "${RWHW_ROOT}/cmake")
if ((NOT ROBWORK_FOUND) OR (NOT ROBWORKHARDWARE_FOUND))
  message(WARNING "Skipping ${PROJECT_NAME}. Requires RobWork and the netft component of RobWorkHardware!!!")
else()
  include_directories(include ${ROBWORKHARDWARE_INCLUDE_DIRS})

###################################
## catkin specific configuration ##
###################################
  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS 
      caros_common 
      caros_common_msgs 
      caros_common_robwork 
      caros_sensor 
      caros_sensor_msgs
      geometry_msgs 
      message_runtime 
      roscpp 
      std_msgs 
)



########################################################################
#### Executables
########################################################################
  include_directories(
    include
    ${ROBWORK_INCLUDE_DIR}
    ${ROBWORKHARDWARE_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

  add_executable(caros_netft_node src/main_netft.cpp src/netft_node.cpp)
  target_link_libraries(caros_netft_node ${conv_library}
    ${ROBWORK_LIBRARIES}
    ${ROBWORKHARDWARE_LIBRARIES}
    ${caros_sensor_LIBRARIES}
    ${caros_common_LIBRARIES}
    ${catkin_LIBRARIES}
  )
  add_dependencies(caros_netft_node
    caros_common
    caros_control
  )
  
  ########################################################################
  #### Testing
  ########################################################################
  if(CATKIN_ENABLE_TESTING)
    add_executable(simple_caros_netft_demo test/simple_netft_demo.cpp)
    target_link_libraries(simple_caros_netft_demo ${catkin_LIBRARIES})
  endif()
endif()
