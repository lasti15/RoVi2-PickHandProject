#ifndef CAROS_ROBOTIQ_ROBOTIQ3_NODE_H
#define CAROS_ROBOTIQ_ROBOTIQ3_NODE_H

#include <caros/caros_node_service_interface.h>
#include <caros/gripper_service_interface.h>

#include <rwhw/robotiq/Robotiq3.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/math/Q.hpp>

#include <ros/ros.h>

namespace caros
{
/**
 * @brief Ros node for controlling Robotiq-3 gripper.
 */
class Robotiq3Node : public caros::CarosNodeServiceInterface, public caros::GripperServiceInterface
{
 public:
  //! constructor
  Robotiq3Node(const ros::NodeHandle& node_handle);

  //! destructor
  virtual ~Robotiq3Node();

  //! @copydoc caros::GripperServiceInterface::moveQ
  bool moveQ(const rw::math::Q& q);

  //! @copydoc caros::GripperServiceInterface::moveQ
  bool gripQ(const rw::math::Q& q);

  //! @copydoc caros::GripperServiceInterface::setForceQ
  bool setForceQ(const rw::math::Q& q);

  //! @copydoc caros::GripperServiceInterface::setVelocityQ
  bool setVelocityQ(const rw::math::Q& q);

  //! @copydoc caros::GripperServiceInterface::stopMovement
  bool stopMovement(void);

  /* TODO: Properly document the error codes */
  /* TODO: Consider better error codes for ROBOTIQ3NODE_INTERNAL_ERROR */
  /* The enum order should not be changed, as recorded ROS sessions would then be invalidated */
  enum ROBOTIQ3NODE_ERRORCODE
  {
    ROBOTIQ3NODE_ROBOTIQ_DEVICE_ALREADY_ACTIVE = 1,
    ROBOTIQ3NODE_CAROS_GRIPPER_SERVICE_CONFIGURE_FAIL,
    ROBOTIQ3NODE_ROBOTIQ_DEVICE_CONNECT_FAILED,
    ROBOTIQ3NODE_INTERNAL_ERROR,
    ROBOTIQ3NODE_ROBOTIQ_DEVICE_NO_CONNECTION,
    ROBOTIQ3NODE_NO_ROBOTIQ_DEVICE,
    ROBOTIQ3NODE_UNSUPPORTED_Q_LENGTH
  };

 protected:
  // hooks implemented from CarosNodeServiceInterface base class
  bool activateHook();
  bool recoverHook(const std::string& error_msg, const int64_t error_code);

  void runLoopHook();
  void errorLoopHook();
  void fatalErrorLoopHook();

  // Utility functions to configure and connect to the Robotiq device
  bool configureRobotiqDevice();
  bool connectToRobotiqDevice();

  // convenience functions
  bool isInWorkingCondition();

 protected:
  ros::Time last_loop_time_;
  rw::math::Q last_Q_;
  std::string ip_;
  int port_;

 private:
  rw::common::Ptr<rwhw::Robotiq3> robotiq3_;
  ros::NodeHandle node_handle_;
};

}  // end namespace

#endif  //#ifndef CAROS_ROBOTIQ_ROBOTIQ3_NODE_H
