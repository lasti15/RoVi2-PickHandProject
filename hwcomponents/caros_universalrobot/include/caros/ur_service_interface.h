#ifndef CAROS_UNIVERSALROBOT_UR_SERVICE_INTERFACE_H
#define CAROS_UNIVERSALROBOT_UR_SERVICE_INTERFACE_H

#include <caros_universalrobot/ur_service_servo_q.h>
#include <caros_universalrobot/ur_service_servo_t.h>
#include <caros_universalrobot/ur_service_empty.h>
#include <caros_universalrobot/ur_service_force_mode_update.h>
#include <caros_universalrobot/ur_service_force_mode_start.h>
#include <caros_universalrobot/ur_service_force_mode_stop.h>

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Wrench6D.hpp>

#include <ros/ros.h>

#define URSERVICEINTERFACE_SUB_NAMESPACE "ur_service_interface"

/* TODO:
 * Should this be placed inside a namespace, and should it be caros, caros::ur or just ur?
 */

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
  virtual bool urForceModeStart(const rw::math::Transform3D<>& refToffset, const rw::math::Q& selection,
                              const rw::math::Wrench6D<>& wrenchTarget, const rw::math::Q& limits) = 0;
  virtual bool urForceModeUpdate(const rw::math::Wrench6D<>& wrenchTarget) = 0;
  virtual bool urForceModeStop() = 0;

 private:
  bool urServoTHandle(caros_universalrobot::ur_service_servo_t::Request& request,
                    caros_universalrobot::ur_service_servo_t::Response& response);

  bool urServoQHandle(caros_universalrobot::ur_service_servo_q::Request& request,
                    caros_universalrobot::ur_service_servo_q::Response& response);

  bool urForceModeStartHandle(caros_universalrobot::ur_service_force_mode_start::Request& request,
                            caros_universalrobot::ur_service_force_mode_start::Response& response);

  bool urForceModeUpdateHandle(caros_universalrobot::ur_service_force_mode_update::Request& request,
                             caros_universalrobot::ur_service_force_mode_update::Response& response);

  bool urForceModeStopHandle(caros_universalrobot::ur_service_force_mode_stop::Request& request,
                           caros_universalrobot::ur_service_force_mode_stop::Response& response);

 protected:
  ros::NodeHandle nodehandle_;

  ros::ServiceServer srvUrServoT_;
  ros::ServiceServer srvUrServoQ_;
  ros::ServiceServer srvUrForceModeStart_;
  ros::ServiceServer srvUrForceModeUpdate_;
  ros::ServiceServer srvUrForceModeStop_;
};

#endif  // include guard
