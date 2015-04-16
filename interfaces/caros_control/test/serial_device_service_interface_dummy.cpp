#include "serial_device_service_interface_dummy.h"
#include <gtest/gtest.h>

SerialDeviceServiceInterfaceDummy::SerialDeviceServiceInterfaceDummy(ros::NodeHandle nodehandle, const bool returnValue,
                                                                     const bool causeError)
    : caros::SerialDeviceServiceInterface(nodehandle),
      returnValue_(returnValue),
      causeError_(causeError),
      causingErrorMsg_("Intentionally causing error - please ignore it")
{
  /* make ROS publish the services */
  if (not SerialDeviceServiceInterface::configureInterface())
  {
    /* Invalid object, since the services weren't published within this constructor */
    throw std::runtime_error("The service interface could not be configured!");
  }
}

SerialDeviceServiceInterfaceDummy::~SerialDeviceServiceInterfaceDummy()
{
  /* Nothing to clean up */
}

const std::string& SerialDeviceServiceInterfaceDummy::getMostRecentFunctionCalled() const
{
  return mostRecentFunctionCalled_;
}

bool SerialDeviceServiceInterfaceDummy::moveLin(const TransformAndSpeedContainer_t& targets)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  if (causeError_)
  {
    throw std::runtime_error(causingErrorMsg_);
  }
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::movePtp(const QAndSpeedContainer_t& targets)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  if (causeError_)
  {
    throw std::runtime_error(causingErrorMsg_);
  }
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::movePtpT(const TransformAndSpeedContainer_t& targets)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  if (causeError_)
  {
    throw std::runtime_error(causingErrorMsg_);
  }
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::moveVelQ(const rw::math::Q& q_vel)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  if (causeError_)
  {
    throw std::runtime_error(causingErrorMsg_);
  }
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::moveVelT(const rw::math::VelocityScrew6D<>& t_vel)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  if (causeError_)
  {
    throw std::runtime_error(causingErrorMsg_);
  }
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::moveLinFc(const rw::math::Transform3D<>& posTarget,
                                                  const rw::math::Transform3D<>& offset,
                                                  const rw::math::Wrench6D<>& wrenchTarget,
                                                  const rw::math::Q& controlGain)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  if (causeError_)
  {
    throw std::runtime_error(causingErrorMsg_);
  }
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::moveServoQ(const QAndSpeedContainer_t& targets)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  if (causeError_)
  {
    throw std::runtime_error(causingErrorMsg_);
  }
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::moveServoT(const TransformAndSpeedContainer_t& targets)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  if (causeError_)
  {
    throw std::runtime_error(causingErrorMsg_);
  }
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::moveStart()
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  if (causeError_)
  {
    throw std::runtime_error(causingErrorMsg_);
  }
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::moveStop()
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  if (causeError_)
  {
    throw std::runtime_error(causingErrorMsg_);
  }
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::movePause()
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  if (causeError_)
  {
    throw std::runtime_error(causingErrorMsg_);
  }
  return returnValue_;
}

bool SerialDeviceServiceInterfaceDummy::moveSetSafeModeEnabled(const bool value)
{
  mostRecentFunctionCalled_ = __PRETTY_FUNCTION__;
  if (causeError_)
  {
    throw std::runtime_error(causingErrorMsg_);
  }
  return returnValue_;
}
