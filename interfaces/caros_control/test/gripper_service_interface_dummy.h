#ifndef CAROS_CONTROL_TEST_GRIPPER_SERVICE_INTERFACE_DUMMY_H
#define CAROS_CONTROL_TEST_GRIPPER_SERVICE_INTERFACE_DUMMY_H

#include <caros/gripper_service_interface.h>

#include <string>
#include <stdexcept>

class GripperServiceInterfaceDummy : public caros::GripperServiceInterface
{
 public:
  GripperServiceInterfaceDummy(ros::NodeHandle nodehandle, const bool returnValue, const bool causeError = false);
  virtual ~GripperServiceInterfaceDummy();

  const std::string& getMostRecentFunctionCalled() const;

  bool moveQ(const rw::math::Q& q);
  bool gripQ(const rw::math::Q& q);
  bool setForceQ(const rw::math::Q& q);
  bool setVelocityQ(const rw::math::Q& q);
  bool stopMovement(void);

 private:
  bool returnValue_;
  bool causeError_;
  std::string mostRecentFunctionCalled_;
};

#endif
