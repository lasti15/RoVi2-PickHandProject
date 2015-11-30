
\mainpage
\htmlinclude manifest.html

[TOC]

This component provides access to a Schunk NetFT Force/Torque sensor.

The Force/Torque sensors measures forces and torques in it local frame of reference. 
Units are Newton and Newton Meter, respectively.

NOTICE: The sensor is not calibrated to have a zero offset. Furthermore the sensor has
a tendency to drift over time. The users of the components need to take care of this 
themselves.


# Interfaces - how to use this node #

## Force Torque Sensor Service Interface ##
The @ref caros::FTSensorServiceInteface interface is fully supported by this node.

The components sends the current readings on the topic *wrench*. The  package type transmitted is a ros geometry_msgs::WrenchStamped. 
On the service interface it provides a rw::math::Wrench6D and a ros::Time

# Requirements #
RobWorkHardware with the *netft* component enabled, is required and can be obtained from http://www.robwork.dk

# Launching the node #
The CAROS NetFT node can be launched by using the following:

    roslaunch caros_netft caros_netft.launch

The launch script is using the default ip and port used in the marvin setup and by default has a publishing rate of 100Hz. This can be changed in the hwcomponents/caros_netft/launch/caros_netft _param.xml file.

## Parameters ##
The following parameters are supported:
| Parameter | Description | Default |
| --------- | ----------- | ------- |
| ip | IP of the netft to read from | 168.192.100.2 |
| port | The modbus communication port used | 49152 |
| rate | Rate (Hz) with which data is published | 100 |

# Small demo(s) #
To quickly and easily verify that the component start up the component using the launch script (see above) and use rqt to view the topic *wrench* and the *caros_node_state*. 


