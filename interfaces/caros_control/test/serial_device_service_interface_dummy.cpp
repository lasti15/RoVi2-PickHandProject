#include "serial_device_service_interface_dummy.h"
#include <gtest/gtest.h>

SerialDeviceServiceInterfaceDummy::SerialDeviceServiceInterfaceDummy(ros::NodeHandle nodehandle, const bool returnValue)
    : caros::SerialDeviceServiceInterface(nodehandle), returnValue_(returnValue)
{
  /* make ROS publish the services */
  SerialDeviceServiceInterface::configureInterface();
}

SerialDeviceServiceInterfaceDummy::~SerialDeviceServiceInterfaceDummy()
{
  /* Nothing to clean up */
}

std::string SerialDeviceServiceInterfaceDummy::getMostRecentFunctionCalled()
{
  return mostRecentFunctionCalled_;
}

bool SerialDeviceServiceInterfaceDummy::moveLin(const TransformAndSpeedContainer_t& targets)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::movePTP(const QAndSpeedContainer_t& targets)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::movePTP_T(const TransformAndSpeedContainer_t& targets)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::moveVelQ(const rw::math::Q& q_vel)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::moveVelT(const rw::math::VelocityScrew6D<>& t_vel)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::moveLinFC(const rw::math::Transform3D<>& posTarget,
                                                  const rw::math::Transform3D<>& offset,
                                                  const rw::math::Wrench6D<>& wrenchTarget,
                                                  const rw::math::Q& controlGain)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::moveServoQ(const QAndSpeedContainer_t& targets)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::moveServoT(const TransformAndSpeedContainer_t& targets)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::moveStart()
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::moveStop()
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::movePause()
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::moveSetSafeModeEnabled(const bool value)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  return returnValue_;
}
