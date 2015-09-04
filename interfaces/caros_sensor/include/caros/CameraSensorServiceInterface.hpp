/**/
#ifndef CAROS_CAMERASENSORSERVICEINTERFACE_HPP
#define CAROS_CAMERASENSORSERVICEINTERFACE_HPP

#include <ros/ros.h>

/**
 * @brief describe the minimum interface of a camera device.
 */
class CameraSensorServiceInterface
{
public:
  //! constructor
  CameraSensorServiceInterface(const std::string& service_name);

protected:
  ros::NodeHandle _nodeHnd;

};

#endif
