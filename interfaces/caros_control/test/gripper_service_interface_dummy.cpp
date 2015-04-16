#include "gripper_service_interface_dummy.h"
#include <gtest/gtest.h>

GripperServiceInterfaceDummy::GripperServiceInterfaceDummy(ros::NodeHandle nodehandle, const bool returnValue,
                                                           const bool causeError)
    : caros::GripperServiceInterface(nodehandle),
      returnValue_(returnValue),
      causeError_(causeError),
      causingErrorMsg_("Intentionally causing error - please ignore it")
{
  /* make ROS publish the services */
  if (not GripperServiceInterface::configureInterface())
  {
    /* Invalid object, since the services weren't published within this constructor */
    throw std::runtime_error("The service interface could not be configured!");
  }
}

GripperServiceInterfaceDummy::~GripperServiceInterfaceDummy()
{
  /* Nothing to clean up */
}

const std::string& GripperServiceInterfaceDummy::getMostRecentFunctionCalled() const
{
  return mostRecentFunctionCalled_;
}

bool GripperServiceInterfaceDummy::moveQ(const rw::math::Q& q)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  if (causeError_)
  {
    throw std::runtime_error(causingErrorMsg_);
  }
  return returnValue_;
}

bool GripperServiceInterfaceDummy::gripQ(const rw::math::Q& q)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  if (causeError_)
  {
    throw std::runtime_error(causingErrorMsg_);
  }
  return returnValue_;
}

bool GripperServiceInterfaceDummy::setForceQ(const rw::math::Q& q)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  if (causeError_)
  {
    throw std::runtime_error(causingErrorMsg_);
  }
  return returnValue_;
}

bool GripperServiceInterfaceDummy::setVelocityQ(const rw::math::Q& q)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  if (causeError_)
  {
    throw std::runtime_error(causingErrorMsg_);
  }
  return returnValue_;
}

bool GripperServiceInterfaceDummy::stopMovement(void)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  if (causeError_)
  {
    throw std::runtime_error(causingErrorMsg_);
  }
  return returnValue_;
}
