#ifndef CAROS_SERIAL_DEVICE_SERVICE_INTERFACE_DUMMY_H
#define CAROS_SERIAL_DEVICE_SERVICE_INTERFACE_DUMMY_H

#include <caros/SerialDeviceServiceInterface.hpp>

#include <string>

class SerialDeviceServiceInterfaceDummy : public caros::SerialDeviceServiceInterface
{
 public:
  SerialDeviceServiceInterfaceDummy(ros::NodeHandle nodehandle, const bool returnValue);
  virtual ~SerialDeviceServiceInterfaceDummy();

  const std::string& getMostRecentFunctionCalled() const;

  bool moveLin(const TransformAndSpeedContainer_t& targets);
  bool movePTP(const QAndSpeedContainer_t& targets);
  bool movePTP_T(const TransformAndSpeedContainer_t& targets);
  bool moveVelQ(const rw::math::Q& q_vel);
  bool moveVelT(const rw::math::VelocityScrew6D<>& t_vel);
  bool moveLinFC(const rw::math::Transform3D<>& posTarget, const rw::math::Transform3D<>& offset,
                 const rw::math::Wrench6D<>& wrenchTarget, const rw::math::Q& controlGain);

  bool moveServoQ(const QAndSpeedContainer_t& targets);
  bool moveServoT(const TransformAndSpeedContainer_t& targets);
  bool moveStart();
  bool moveStop();
  bool movePause();
  bool moveSetSafeModeEnabled(const bool value);

 private:
  bool returnValue_;
  std::string mostRecentFunctionCalled_;
};

#endif
