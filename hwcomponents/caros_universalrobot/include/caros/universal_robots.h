#ifndef CAROS_UNIVERSALROBOT_UNIVERSAL_ROBOTS_H
#define CAROS_UNIVERSALROBOT_UNIVERSAL_ROBOTS_H

#include <caros/ur_service_interface.h>

#include <caros/caros_node_service_interface.h>
#include <caros/serial_device_service_interface.h>

#include <caros_common_msgs/WrenchData.h>

#include <rw/invkin/JacobianIKSolver.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Wrench6D.hpp>

#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>

#include <queue>

#define SUPPORTED_Q_LENGTH_FOR_UR 6

namespace caros
{
class UniversalRobots : public caros::CarosNodeServiceInterface,
                        public caros::SerialDeviceServiceInterface,
                        public URServiceInterface
{
 public:
  UniversalRobots(const ros::NodeHandle& nodehandle);

  virtual ~UniversalRobots();

  enum URNODE_ERRORCODE
  {
    URNODE_MISSING_PARAMETER = 1,
    URNODE_URSERVICE_CONFIGURE_FAIL,
    URNODE_SERIALDEVICESERVICE_CONFIGURE_FAIL,
    URNODE_MISSING_WORKCELL,
    URNODE_NO_SUCH_DEVICE_IN_WORKCELL,
    URNODE_FAULTY_SUBSCRIBER,
    URNODE_INVALID_CALLBACKPORT,
    URNODE_UNSUCCESSFUL_CONNECT_TO_URRT,
    URNODE_UNSUCCESSFUL_CONNECT_TO_UR,
    URNODE_UNSUPPORTED_Q_LENGTH,
    URNODE_INTERNAL_ERROR
  };

  /************************************************************************
   * URServiceInterface functions
   ************************************************************************/
  //! @copydoc URServiceInterface::servoT
  bool urServoT(const rw::math::Transform3D<>& target);
  //! @copydoc URServiceInterface::servoQ
  bool urServoQ(const rw::math::Q& target);
  //! @copydoc URServiceInterface::forceModeStart
  bool urForceModeStart(const rw::math::Transform3D<>& ref_t_offset, const rw::math::Q& selection,
                        const rw::math::Wrench6D<>& wrench_target, const rw::math::Q& limits);
  //! @copydoc URServiceInterface::forceModeUpdate
  bool urForceModeUpdate(const rw::math::Wrench6D<>& wrench_target);
  //! @copydoc URServiceInterface::forceModeStop
  bool urForceModeStop();

  /************************************************************************
   * SerialDeviceServiceInterface functions
   ************************************************************************/
  //! @copydoc caros::SerialDeviceServiceInterface::moveLin
  bool moveLin(const TransformAndSpeedContainer_t& targets);
  //! @copydoc caros::SerialDeviceServiceInterface::movePtp
  bool movePtp(const QAndSpeedContainer_t& targets);
  //! @copydoc caros::SerialDeviceServiceInterface::movePtpT
  bool movePtpT(const TransformAndSpeedContainer_t& targets);
  //! @copydoc caros::SerialDeviceServiceInterface::moveVelQ
  bool moveVelQ(const rw::math::Q& q_vel);
  //! @copydoc caros::SerialDeviceServiceInterface::moveVelT
  bool moveVelT(const rw::math::VelocityScrew6D<>& t_vel);
  //! @copydoc caros::SerialDeviceServiceInterface::moveLinFc
  bool moveLinFc(const rw::math::Transform3D<>& pos_target, const rw::math::Transform3D<>& offset,
                 const rw::math::Wrench6D<>& wrench_target, const rw::math::Q& control_gain);
  //! @copydoc caros::SerialDeviceServiceInterface::moveServoQ
  bool moveServoQ(const QAndSpeedContainer_t& targets);
  //! @copydoc caros::SerialDeviceServiceInterface::moveServoT
  bool moveServoT(const TransformAndSpeedContainer_t& targets);
  //! @copydoc caros::SerialDeviceServiceInterface::moveStart
  bool moveStart();
  //! @copydoc caros::SerialDeviceServiceInterface::moveStop
  bool moveStop();
  //! @copydoc caros::SerialDeviceServiceInterface::movePause
  bool movePause();
  //! @copydoc caros::SerialDeviceServiceInterface::moveSetSafeModeEnabled
  bool moveSetSafeModeEnabled(const bool value);

 protected:
  /************************************************************************
   * Hooks implemented from CarosNodeServiceInterface base class
   ************************************************************************/
  bool activateHook();
  bool recoverHook(const std::string& error_msg, const int64_t error_code);

  void runLoopHook();
  void errorLoopHook();
  void fatalErrorLoopHook();

 private:
  //! @brief Support function for capturing published wrench data
  void addFTData(const caros_common_msgs::WrenchData::ConstPtr state);

  /* convenience functions */
  bool isInWorkingCondition();
  bool supportedQSize(const rw::math::Q& q);

 private:
  ros::NodeHandle nodehandle_;

  rw::models::WorkCell::Ptr workcell_;
  rw::models::Device::Ptr device_;
  rw::math::Q qcurrent_; /* Updated in runLoopHook() */
  rw::kinematics::Frame* ft_frame_;

  rwhw::URCallBackInterface ur_;
  rwhw::UniversalRobotsRTLogging urrt_;

  // [ not being initialised ] rwhw::NetFTLogging::Ptr p_net_ft_;
  //    rwhw::FTCompensation::Ptr p_ft_compensation_;
  ros::Subscriber sub_ft_data_;
  std::queue<rw::math::Wrench6D<>> wrench_data_queue_;

  rw::invkin::JacobianIKSolver::Ptr ik_solver_;
  rw::kinematics::State state_;

  bool use_ft_collision_detection_;
};
}
#endif  // include guard
