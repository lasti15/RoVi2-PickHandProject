\mainpage
<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc/generate-toc again -->
**Table of Contents**

- [caros_control](#caroscontrol)

<!-- markdown-toc end -->

# caros_control #
This component contains interfaces for controlling various types of devices. Examples hereof are gripper or a robot arm. See [classes](annotated.html) for a list of the available control service interfaces and corresponding proxies.

# Interfaces #
All interfaces in here are abstract and parts need to be added by the user for the specific device. All proxies are useable directly

## Gripper Service Interface ##
The @ref caros::GripperServiceInterface represents a simple gripper interface. It has one output topic:
| Topic | Type | Description |
| ------- | :------: | :-------------: |
| gripper_state  | @ref caros_control_msgs::GripperState | State of the gripper (joint pos, joint vel., ...)|

In addition the following services are provided.

| Service | Type | Description |
| ------- | :------: | :-------------: |
| move_q  | @ref caros_control_msgs::GripperMoveQ | Move the gripper to the given target Q (and then switch off power if applicable)|
| grip_q  | @ref caros_control_msgs::GripperGripQ | Move the gripper to the given target Q and keep powered (grip/hold the object in hand)|
| set_force_q  | @ref caros_control_msgs::GripperSetForceQ | Set the max gripping force according to the given Q|
| set_velocity_q  | @ref caros_control_msgs::GripperSetVelocityQ | Set the target velocity according to the given Q|
| stop_movement  | @ref caros_control_msgs::GripperStopMovement | Stop the gripper movement (if the device supports this)|


## Serial Device Service Interface ##
The @ref caros::SerialDeviceServiceInterface represents a interface for a serial device (e.g., a articulated robot arm). It has one output topic:
| Topic | Type | Description |
| ------- | :------: | :-------------: |
| robot_state  | @ref caros_control_msgs::RobotState | State of the robot (joint pos, joint velocity, ...)|

In addition the following services are provided.

| Service | Type | Description |
| ------- | :------: | :-------------: |
| move_lin | @ref caros_control_msgs::SerialDeviceMoveLin |  |
| move_ptp | @ref caros_control_msgs::SerialDeviceMovePtp |  |
| move_ptp | @ref caros_control_msgs::SerialDeviceMovePtp |  |
| move_ptp_t | @ref caros_control_msgs::SerialDeviceMovePtpT |  |
| move_vel_q | @ref caros_control_msgs::SerialDeviceMoveVelQ |  |
| move_vel_t | @ref caros_control_msgs::SerialDeviceMoveVelT |  |
| move_servo_q | @ref caros_control_msgs::SerialDeviceMoveServoQ |  |
| move_servo_t | @ref caros_control_msgs::SerialDeviceMoveServoT |  |
| move_lin_fc | @ref caros_control_msgs::SerialDeviceMoveLinFc |  |
| move_start | @ref caros_control_msgs::SerialDeviceForceControlStart |  |
| move_stop | @ref caros_control_msgs::SerialDeviceForceControlStop |  |
| move_pause | @ref caros_control_msgs:: |  |
|  | @ref caros_control_msgs::SerialDeviceForceControlUpdate |  |

