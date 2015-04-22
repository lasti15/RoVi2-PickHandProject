\mainpage

[TOC]

caros_universalrobot is a ROS node for controlling a physical UniversalRobot. A few interfaces are available for controlling the robot.

# Interfaces - how to use this node #
A few interfaces (i.e. ways to control the robot through ROS services) are supported.
## caros::SerialDeviceServiceInterface ##
The @ref caros::SerialDeviceServiceInterface interface is supported to some extent. The functionalities that have not been tested or verified are disabled (i.e. they will not respond with a proper acknowledgment and output a ROS ERROR message). The status of the missing services can be seen below:
| Service | Disabled | Not Implemented |
| ------- | :------: | :-------------: |
| move_vel_q  | x |   |
| move_vel_t  |   | x |
| move_lin_fc | x |   |
| move_start  |   | x |
| move_pause  |   | x |
| set_safe_mode_enabled |   | x |

## URServiceInterface ##
The @ref URServiceInterface interface is supported, but most of the functionalities, especially the force mode functions, have not been tested yet.

# Requirements #
RobWorkHardware with the *universalrobots* component enabled, is required and can be obtained from http://www.robwork.dk

# Launching the node #
The CAROS UniversalRobot node can be launched by using the following:

    roslaunch caros_universalrobot caros_ur.launch

Currently the launch script is complaining if no scene (workcell) is provided. The scene should be specified on the parameter server according to @ref caros::getWorkCell. To use the scene specified through the parameter server, then use:

    roslaunch caros_universalrobot caros_ur.launch set_workcell:=0

Or specify the scene to use, when launching the node:

    roslaunch caros_universalrobot caros_ur.launch workcell_path:=/path/to/the/scene_or_workcell.xml

## Parameters ##
The following parameters are supported:
| Parameter | Description |
| --------- | ----------- |
| deviceName | The name of the robot device within the scene |
| FTFrame | The name of the force/torque frame in the scene |
| IP | IP of the robot to control |
| callbackPort | Port (on the computer/host) to be used for communicating with the robot |
| callbackIP | The IP of the computer/host that should communicate with the robot |
| WrenchTopic | Name of the topic to subscribe to for getting wrench data (to be used with force/torque mode) |

# Small demo(s) #
To quickly and easily verify that the communication with the robot is working, then there are one or more simple demos that can be run. The expected behaviour should be both observed and verified by the user.
## Available demo(s) ##
| Demo | Expected behaviour | Notes |
| ---- | ------------------ | ----- |
| simple_demo_using_serial_device_sip | Moving the robot arm forth and back linearly in the joint-configuration space. | None |

### simple_demo_using_serial_device_sip ###
| Parameter | Description |
| --------- | ----------- |
| qChange | The overall change in the joint-configuration space |
| steps | The number of steps to take while traversing the path |
| movements | The number of repetitions to perform |

## Launching the demo(s) ##
In order to make ROS properly find the demos, then the <your_catkin_workspace>/devel/setup.bash file should be sourced. If standing in your catkin workspace then it's as simple as (if you are using BASH or similar shell - default on Ubuntu):

    source devel/setup.bash

To launch the demos:

    roslaunch caros_universalrobot <demo name>.test

For example to launch the simple_demo_using_serial_device_sip:

    roslaunch caros_universalrobot simple_demo_using_serial_device_sip.test

### Using debug verbosity ###
To enable debug verbosity and thus hopefully make it easier to diagnose issues, then a rosconsole debug configuration file has to be present (see https://gitlab.com/caro-sdu/caros/wikis/Tests#example-rosconsole_debug-conf):

    ROSCONSOLE_CONFIG_FILE=/path/to/rosconsole_debug.conf roslaunch caros_universalrobot simple_demo_using_serial_device_sip.test
