cmake_minimum_required(VERSION 2.8.3)
project(caros_universalrobot)

########################################################################
#### Local Variables
########################################################################
set(node_name "${PROJECT_NAME}_node")

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
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs roscpp rosconsole caros_common_msgs caros_common caros_common_robwork caros_control caros_control_msgs)

########################################################################
#### RobWork and RobWorkSim
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
set(RWHW_ROOT "$ENV{RWHW_ROOT}")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")
find_package(RobWorkHardware COMPONENTS universalrobots PATHS "${RWHW_ROOT}")

if (NOT ROBWORKHARDWARE_FOUND)
  message(WARNING "Skipping ${PROJECT_NAME}. Requires the universalrobots component of RobWorkHardware!")
else()
  ################################################
  ## Declare ROS messages, services and actions ##
  ################################################
  add_service_files(
    FILES
    ur_service_servo_q.srv
    ur_service_servo_t.srv
    ur_service_empty.srv
    ur_service_force_mode_start.srv
    ur_service_force_mode_update.srv
    ur_service_force_mode_stop.srv
    )

  generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    caros_common_msgs
    )

  ###################################
  ## catkin specific configuration ##
  ###################################
  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime std_msgs geometry_msgs roscpp caros_common_msgs caros_common caros_common_robwork caros_control caros_control_msgs
    DEPENDS RobWork RobWorkHardware
    CFG_EXTRAS "use-c++11.cmake"
    )

  ########################################################################
  #### Build
  ########################################################################
  include_directories(
    include
    ${ROBWORK_INCLUDE_DIR}
    ${ROBWORKHARDWARE_INCLUDE_DIRS}
    ${caros_control_INCLUDE_DIRS}
    ${caros_common_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

  add_executable(${node_name} src/ur_main.cpp src/universal_robots.cpp src/ur_service_interface.cpp)
  target_link_libraries(${node_name}
    ${ROBWORK_LIBRARIES}
    ${ROBWORKHARDWARE_LIBRARIES}
    ${caros_control_LIBRARIES}
    ${caros_common_LIBRARIES}
    ${catkin_LIBRARIES}
    )

  add_dependencies(${node_name}
    caros_common
    caros_common_robwork
    caros_control
    )

  ########################################################################
  #### Testing
  ########################################################################
  if(CATKIN_ENABLE_TESTING)
    add_executable(simple_${PROJECT_NAME}_demo_using_serial_device_sip test/simple_demo_using_serial_device_sip.cpp)
    target_link_libraries(simple_${PROJECT_NAME}_demo_using_serial_device_sip ${catkin_LIBRARIES})
  endif()

endif()
