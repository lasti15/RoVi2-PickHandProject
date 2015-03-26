cmake_minimum_required(VERSION 2.8.3)
project(caros_control_msgs)

########################################################################
#### Local Variables
########################################################################

########################################################################
#### Catkin Packages
########################################################################
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs caros_common_msgs)

################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(
  FILES
  serial_device_force_control_start.srv
  serial_device_force_control_stop.srv
  serial_device_force_control_update.srv
  serial_device_move_lin.srv
  serial_device_move_lin_fc.srv
  serial_device_move_ptp_t.srv
  serial_device_move_ptp.srv
  serial_device_move_vel_q.srv
  serial_device_move_vel_t.srv
  serial_device_move_servo_q.srv
  serial_device_move_servo_t.srv
  gripper_move_q.srv
  gripper_grip_q.srv
  gripper_set_force_q.srv
  gripper_set_velocity_q.srv
  gripper_stop_movement.srv
)

add_message_files(
  FILES
  robot_state.msg
  gripper_state.msg
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
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs caros_common_msgs
)
