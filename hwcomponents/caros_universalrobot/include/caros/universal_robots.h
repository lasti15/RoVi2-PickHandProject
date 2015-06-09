#ifndef CAROS_UNIVERSALROBOT_UNIVERSAL_ROBOTS_H
#define CAROS_UNIVERSALROBOT_UNIVERSAL_ROBOTS_H

#include <caros/ur_service_interface.h>

#include <caros/caros_node_service_interface.h>
#include <caros/serial_device_service_interface.h>

#include <caros_common_msgs/wrench_data.h>

#include <rw/invkin/JacobianIKSolver.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Wrench6D.hpp>

#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>

#include <queue>

namespace caros
{
class UniversalRobots : public caros::CarosNodeServiceInterface,
                        public caros::SerialDeviceServiceInterface,
                        public URServiceInterface
{
 public:
  UniversalRobots(const ros::NodeHandle& nodehandle, rw::models::WorkCell::Ptr workcell);

  virtual ~UniversalRobots();

  enum URNODE_ERRORCODE
  {
    URNODE_MISSING_PARAMETER = 1,
    URNODE_URSERVICE_CONFIGURE_FAIL,
    URNODE_SERIALDEVICESERVICE_CONFIGURE_FAIL,
    URNODE_MISSING_WORKCELL,
    URNODE_NO_SUCH_DEVICE,
    URNODE_NO_SUCH_FRAME,
    URNODE_FAULTY_SUBSCRIBER
  };

  /************************************************************************
   * URServiceInterface functions
   ************************************************************************/
  //! @copydoc URServiceInterface::servoT
  bool urServoT(const rw::math::Transform3D<>& target);
  //! @copydoc URServiceInterface::servoQ
  bool urServoQ(const rw::math::Q& target);
  //! @copydoc URServiceInterface::forceModeStart
  bool urForceModeStart(const rw::math::Transform3D<>& refToffset, const rw::math::Q& selection,
                      const rw::math::Wrench6D<>& wrenchTarget, const rw::math::Q& limits);
  //! @copydoc URServiceInterface::forceModeUpdate
  bool urForceModeUpdate(const rw::math::Wrench6D<>& wrenchTarget);
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
  bool moveLinFc(const rw::math::Transform3D<>& posTarget, const rw::math::Transform3D<>& offset,
                 const rw::math::Wrench6D<>& wrenchTarget, const rw::math::Q& controlGain);
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
  bool recoverHook();

  void runLoopHook();
  void errorLoopHook();
  void fatalErrorLoopHook();

 private:
  //! @brief Support function for capturing published wrench data
  void addFTData(const caros_common_msgs::wrench_data::ConstPtr state);

 private:
  ros::NodeHandle nodehandle_;

  rw::models::WorkCell::Ptr workcell_;
  rw::models::Device::Ptr device_;
  rw::math::Q qcurrent_; /* Updated in runLoopHook() */
  rw::kinematics::Frame* ftFrame_;

  rwhw::URCallBackInterface ur_;        /* shared */
  rwhw::UniversalRobotsRTLogging urrt_; /* shared */

  // [ not being initialised ] rwhw::NetFTLogging::Ptr pNetFT_;
  //    rwhw::FTCompensation::Ptr pFTCompensation_;
  ros::Subscriber subFTData_;
  std::queue<rw::math::Wrench6D<>> wrenchDataQueue_;

  rw::invkin::JacobianIKSolver::Ptr iksolver_;
  rw::kinematics::State
      state_; /* Updated as needed by calling device_->setQ(qcurrent_, state_) or some other q-configuration */

  bool useFTCollisionDetection_;
};
}
#endif  // include guard
