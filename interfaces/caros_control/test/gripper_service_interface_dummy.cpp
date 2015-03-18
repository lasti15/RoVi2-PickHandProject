#include "gripper_service_interface_dummy.h"
#include <gtest/gtest.h>

GripperServiceInterfaceDummy::GripperServiceInterfaceDummy(ros::NodeHandle nodehandle, const bool returnValue)
    : caros::GripperServiceInterface(nodehandle), returnValue_(returnValue)
{
  /* make ROS publish the services */
  GripperServiceInterface::configureGripperService();
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
  return returnValue_;
}

bool GripperServiceInterfaceDummy::gripQ(const rw::math::Q& q)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  return returnValue_;
}

bool GripperServiceInterfaceDummy::setForceQ(const rw::math::Q& q)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  return returnValue_;
}

bool GripperServiceInterfaceDummy::setVelocityQ(const rw::math::Q& q)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  return returnValue_;
}

bool GripperServiceInterfaceDummy::stopMovement(void)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  return returnValue_;
}
