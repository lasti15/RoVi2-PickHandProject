/**/
#ifndef CAROS_CAMERASENSORSERVICEINTERFACE_HPP
#define CAROS_CAMERASENSORSERVICEINTERFACE_HPP

#include <rw/common/Ptr.hpp>

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
  rw::common::Ptr<ros::NodeHandle> _nodeHnd;

};

#endif
