/**/
#ifndef CAROS_STEREOCAMERASENSORSERVICEINTERFACE_HPP
#define CAROS_STEREOCAMERASENSORSERVICEINTERFACE_HPP

#include <ros/ros.h>

/**
 * @brief describe the minimum interface of a stereo camera device.
 */
class StereoCameraSensorServiceInterface {
public:
	StereoCameraSensorServiceInterface(const std::string& service_name);

protected:
	rw::common::Ptr<ros::NodeHandle> _nodeHnd;
};

#endif
