#ifndef CAROS_UNIVERSALROBOT_UR_SERVICE_INTERFACE_H
#define CAROS_UNIVERSALROBOT_UR_SERVICE_INTERFACE_H

#include <caros_universalrobot/UrServiceServoQ.h>
#include <caros_universalrobot/UrServiceServoT.h>
#include <caros_universalrobot/UrServiceEmpty.h>
#include <caros_universalrobot/UrServiceForceModeUpdate.h>
#include <caros_universalrobot/UrServiceForceModeStart.h>
#include <caros_universalrobot/UrServiceForceModeStop.h>

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Wrench6D.hpp>

#include <ros/ros.h>

#define URSERVICEINTERFACE_SUB_NAMESPACE "ur_service_interface"

class URServiceInterface
{
 public:
  /**
   * @brief constructor
   * @param[in] nodehandle the nodehandle to use for services.
   */
  URServiceInterface(const ros::NodeHandle& nodehandle);

  /**
   * @brief virtual destructor
   */
  virtual ~URServiceInterface();

  /**
   * @brief setup the ROS services for this interface
   */
  bool configureInterface();

  /**
   * @brief move robot using a pose as target (requires inverse kinematics)
   */
  virtual bool urServoT(const rw::math::Transform3D<>& target) = 0;

  /**
   * @brief move robot using an Q-configuration as target
   */
  virtual bool urServoQ(const rw::math::Q& target) = 0;

  /* TODO:
   * 'force mode stop' doesn't seem to do anything within the
   * RobWorkHardware/src/rwhw/universalrobot/URCallBackInterface.cpp file...
   */
  /* FIXME:
   * Properly document what these functions do and when/why they should be used
   */
  virtual bool urForceModeStart(const rw::math::Transform3D<>& ref_t_offset, const rw::math::Q& selection,
                                const rw::math::Wrench6D<>& wrench_target, const rw::math::Q& limits) = 0;
  virtual bool urForceModeUpdate(const rw::math::Wrench6D<>& wrench_target) = 0;
  virtual bool urForceModeStop() = 0;

 private:
  bool urServoTHandle(caros_universalrobot::UrServiceServoT::Request& request,
                      caros_universalrobot::UrServiceServoT::Response& response);

  bool urServoQHandle(caros_universalrobot::UrServiceServoQ::Request& request,
                      caros_universalrobot::UrServiceServoQ::Response& response);

  bool urForceModeStartHandle(caros_universalrobot::UrServiceForceModeStart::Request& request,
                              caros_universalrobot::UrServiceForceModeStart::Response& response);

  bool urForceModeUpdateHandle(caros_universalrobot::UrServiceForceModeUpdate::Request& request,
                               caros_universalrobot::UrServiceForceModeUpdate::Response& response);

  bool urForceModeStopHandle(caros_universalrobot::UrServiceForceModeStop::Request& request,
                             caros_universalrobot::UrServiceForceModeStop::Response& response);

 protected:
  ros::NodeHandle nodehandle_;

  ros::ServiceServer srv_ur_servo_t_;
  ros::ServiceServer srv_ur_servo_q_;
  ros::ServiceServer srv_ur_force_mode_start_;
  ros::ServiceServer srv_ur_force_mode_update_;
  ros::ServiceServer srv_ur_force_mode_stop_;
};

#endif  // include guard
