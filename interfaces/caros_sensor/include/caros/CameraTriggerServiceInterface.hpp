/**/
#ifndef CAROS_CAMERATRIGGERSERVICEINTERFACE_HPP
#define CAROS_CAMERATRIGGERSERVICEINTERFACE_HPP

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
  ros::NodeHandle _nodeHnd;
};

#endif
