#ifndef CAROS_GRIPPER_SERVICE_INTERFACE_DUMMY_H
#define CAROS_GRIPPER_SERVICE_INTERFACE_DUMMY_H

#include <caros/GripperServiceInterface.hpp>

#include <string>

class GripperServiceInterfaceDummy : public caros::GripperServiceInterface
{
 public:
  GripperServiceInterfaceDummy(ros::NodeHandle nodehandle, const bool returnValue);
  virtual ~GripperServiceInterfaceDummy();

  std::string getMostRecentFunctionCalled();

  bool moveQ(const rw::math::Q& q);
  bool gripQ(const rw::math::Q& q);
  bool setForceQ(const rw::math::Q& q);
  bool setVelocityQ(const rw::math::Q& q);
  bool stopMovement(void);

 private:
  bool returnValue_;
  std::string mostRecentFunctionCalled_;
};

#endif
