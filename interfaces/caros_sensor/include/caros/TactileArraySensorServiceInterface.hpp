/**/
#ifndef CAROS_TACTILEARRAYSENSORSERVICEINTERFACE_HPP
#define CAROS_TACTILEARRAYSENSORSERVICEINTERFACE_HPP

#include <ros/ros.h>

/**
 * @brief describe the minimum interface of a tactile array device.
 */
class TactileArraySensorServiceInterface {
public:
	TactileArraySensorServiceInterface(const std::string& service_name);

protected:
	rw::common::Ptr<ros::NodeHandle> _nodeHnd;
};

#endif
