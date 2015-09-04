/**/
#ifndef CAROS_KINECTSENSORSERVICEINTERFACE_HPP
#define CAROS_KINECTSENSORSERVICEINTERFACE_HPP

#include <ros/ros.h>

/**
 * @brief describe the minimum interface of a Kinect device.
 */
class KinectSensorServiceInterface
{
public:
  //! constructor
  KinectSensorServiceInterface(const std::string& service_name);

protected:
  ros::NodeHandle _nodeHnd;
};

#endif
