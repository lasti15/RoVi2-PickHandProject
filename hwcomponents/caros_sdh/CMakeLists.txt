cmake_minimum_required(VERSION 2.8.3)
project(caros_sdh)

########################################################################
#### Local Variables
########################################################################
set(library_name ${PROJECT_NAME})

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS roscpp rosconsole caros_common caros_control)

########################################################################
#### RobWork and RobWorkHardware
########################################################################
set(RW_ROOT "$ENV{RW_ROOT}")
set(RWHW_ROOT "$ENV{RWHW_ROOT}")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")
find_package(RobWorkHardware COMPONENTS sdh serialport PATHS "${RWHW_ROOT}")

if (NOT ROBWORKHARDWARE_FOUND)
  message(WARNING "Skipping ${PROJECT_NAME}. Requires both SDH and Serialport components of RobWorkHardware!")
else()
  ########################################################################
  #### SDH and CAN
  ########################################################################
  # find the SDH library to use
  find_package(SDH REQUIRED)
  # we use peak so find the library to use
  find_package(PEAKCAN)

  if(NOT PEAKCAN_FOUND)
    set(PEAKCAN_INCLUDE_DIR "")
    set(PEAKCAN_LIBRARIES "")
    message(STATUS "The PEAK CAN libraries have not been found. You will only be able to use RS232")
  endif()

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
  #   std_msgs
  # )

  ###################################
  ## catkin specific configuration ##
  ###################################
  catkin_package(
    INCLUDE_DIRS include
    # CATKIN_DEPENDS:
    # - rosconsole is not used in the header files, so should not be added to the list here.
    CATKIN_DEPENDS roscpp caros_common caros_control
    DEPENDS RobWork RobWorkHardware
    )

  ########################################################################
  #### Build
  ########################################################################
  include_directories(
    include
    ${ROBWORK_INCLUDE_DIR}
    ${ROBWORKHARDWARE_INCLUDE_DIRS}
    ${PEAKCAN_INCLUDE_DIR}
    ${SDH_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    )

  link_directories(${ROBWORKHARDWARE_LIBRARY_DIRS})

  ########################################################################
  #### SDH Node
  ########################################################################
  add_executable(sdh_node src/main_SDH.cpp src/SDHNode.cpp)
  target_link_libraries(sdh_node
    ${ROBWORK_LIBRARIES}
    ${ROBWORKHARDWARE_LIBRARIES}
    ${SDH_LIBRARY}
    ${PEAKCAN_LIBRARIES}
    ${caros_common_LIBRARIES}
    ${caros_control_LIBRARIES}
    ${catkin_LIBRARIES}
    )
  # Add dependencies on the caros_common and caros_control targets (those are the make-targets for producing the libraries that are being linked with)
  add_dependencies(sdh_node
    caros_common
    caros_control
    )

  ########################################################################
  #### Testing
  ########################################################################
  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(simple_demo test/simple_demo.test test/simple_demo.cpp)
    target_link_libraries(simple_demo ${catkin_LIBRARIES})
  endif()

endif()
