/**/
#ifndef CAROS_KINECTSENSORSERVICEINTERFACE_HPP
#define CAROS_KINECTSENSORSERVICEINTERFACE_HPP

#include <ros/ros.h>

/**
 * @brief describe the minimum interface of a Kinect device.
 */
class KinectSensorServiceInterface {
public:
	KinectSensorServiceInterface(const std::string& service_name);

protected:
	rw::common::Ptr<ros::NodeHandle> _nodeHnd;

#endif
