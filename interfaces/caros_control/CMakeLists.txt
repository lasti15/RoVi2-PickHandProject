cmake_minimum_required(VERSION 2.8.3)
project(caros_control)

########################################################################
#### Local Variables
########################################################################
set(library_name "${PROJECT_NAME}")

########################################################################
#### Make use of c++11
########################################################################
# See: http://answers.ros.org/question/173947/export-gcc-flags-though-catkin-cfg_extras/
# Using hardcoded path-strings instead of a variable substitution for the use-c++11.cmake path-name, as catkin_lint doesn't properly do variable substitution (at least in this particular situation)
include("cmake/use-c++11.cmake")


########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp geometry_msgs caros_common_msgs caros_common caros_control_msgs)

########################################################################
#### RobWork
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")

################################################
## Declare ROS messages, services and actions ##
################################################
# add_service_files(
#   FILES
# )

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
  CATKIN_DEPENDS message_runtime std_msgs roscpp geometry_msgs caros_common_msgs caros_common caros_control_msgs
  DEPENDS RobWork
  CFG_EXTRAS "use-c++11.cmake"
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
  src/gripper_service_interface.cpp
  src/serial_device_service_interface.cpp
  src/gripper_si_proxy.cpp
  src/serial_device_si_proxy.cpp
)

add_dependencies(${library_name}
# ${catkin_EXPORTED_TARGETS} should be used if this target depends on things (e.g. message or service headers) that are imported from other catkin packages. (see http://docs.ros.org/hydro/api/catkin/html/howto/building_msgs.html) - The dependency targets can also be specified manually as is done here for the caros_common_msgs and caros_control_msgs dependencies.
  caros_common_msgs_generate_messages_cpp
  caros_control_msgs_generate_messages_cpp
)

target_link_libraries(${library_name}
  ${catkin_LIBRARIES}
  ${ROBWORK_LIBRARIES}
)

########################################################################
#### Testing
########################################################################
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Template for easy emacs macro generation
  # add_rostest_gtest(#1# test/#1#.test test/#1#.cpp test/serial_device_service_interface_dummy.cpp)
  # target_link_libraries(#1# ${library_name} ${catkin_LIBRARIES})
  # # SerialDeviceSIProxy
  add_rostest_gtest(serial_device_service_interface_proxy_services_success test/serial_device_service_interface_proxy_services_success.test test/serial_device_service_interface_proxy_services_success.cpp test/serial_device_service_interface_dummy.cpp)
  target_link_libraries(serial_device_service_interface_proxy_services_success ${library_name} ${catkin_LIBRARIES})

  add_rostest_gtest(serial_device_service_interface_proxy_services_failure test/serial_device_service_interface_proxy_services_failure.test test/serial_device_service_interface_proxy_services_failure.cpp test/serial_device_service_interface_dummy.cpp)
  target_link_libraries(serial_device_service_interface_proxy_services_failure ${library_name} ${catkin_LIBRARIES})

  add_rostest_gtest(serial_device_service_interface_proxy_bad_service_call test/serial_device_service_interface_proxy_bad_service_call.test test/serial_device_service_interface_proxy_bad_service_call.cpp test/serial_device_service_interface_dummy.cpp)
  target_link_libraries(serial_device_service_interface_proxy_bad_service_call ${library_name} ${catkin_LIBRARIES})

  add_rostest_gtest(serial_device_service_interface_proxy_unavailable_service test/serial_device_service_interface_proxy_unavailable_service.test test/serial_device_service_interface_proxy_unavailable_service.cpp test/serial_device_service_interface_dummy.cpp)
  target_link_libraries(serial_device_service_interface_proxy_unavailable_service ${library_name} ${catkin_LIBRARIES})

  add_rostest_gtest(serial_device_service_interface_proxy_persistent_connection test/serial_device_service_interface_proxy_persistent_connection.test test/serial_device_service_interface_proxy_persistent_connection.cpp test/serial_device_service_interface_dummy.cpp)
  target_link_libraries(serial_device_service_interface_proxy_persistent_connection ${library_name} ${catkin_LIBRARIES})

  # GripperSIProxy
  add_rostest_gtest(gripper_service_interface_proxy_services_success test/gripper_service_interface_proxy_services_success.test test/gripper_service_interface_proxy_services_success.cpp test/gripper_service_interface_dummy.cpp)
  target_link_libraries(gripper_service_interface_proxy_services_success ${library_name} ${catkin_LIBRARIES})

  add_rostest_gtest(gripper_service_interface_proxy_services_failure test/gripper_service_interface_proxy_services_failure.test test/gripper_service_interface_proxy_services_failure.cpp test/gripper_service_interface_dummy.cpp)
  target_link_libraries(gripper_service_interface_proxy_services_failure ${library_name} ${catkin_LIBRARIES})

  add_rostest_gtest(gripper_service_interface_proxy_bad_service_call test/gripper_service_interface_proxy_bad_service_call.test test/gripper_service_interface_proxy_bad_service_call.cpp test/gripper_service_interface_dummy.cpp)
  target_link_libraries(gripper_service_interface_proxy_bad_service_call ${library_name} ${catkin_LIBRARIES})

  add_rostest_gtest(gripper_service_interface_proxy_unavailable_service test/gripper_service_interface_proxy_unavailable_service.test test/gripper_service_interface_proxy_unavailable_service.cpp test/gripper_service_interface_dummy.cpp)
  target_link_libraries(gripper_service_interface_proxy_unavailable_service ${library_name} ${catkin_LIBRARIES})

  add_rostest_gtest(gripper_service_interface_proxy_persistent_connection test/gripper_service_interface_proxy_persistent_connection.test test/gripper_service_interface_proxy_persistent_connection.cpp test/gripper_service_interface_dummy.cpp)
  target_link_libraries(gripper_service_interface_proxy_persistent_connection ${library_name} ${catkin_LIBRARIES})

endif()
