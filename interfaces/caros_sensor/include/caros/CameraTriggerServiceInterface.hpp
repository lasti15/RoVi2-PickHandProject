/**/
#ifndef CAROS_CAMERATRIGGERSERVICEINTERFACE_HPP
#define CAROS_CAMERATRIGGERSERVICEINTERFACE_HPP

#include <rw/common/Ptr.hpp>

#include <ros/ros.h>

/**
 * @brief describe the minimum interface of a camera trigger device.
 */
class CameraTriggerServiceInterface
{
public:
  //! constructor
  CameraTriggerServiceInterface(const std::string& service_name);

protected:
  rw::common::Ptr<ros::NodeHandle> _nodeHnd;
};

#endif
