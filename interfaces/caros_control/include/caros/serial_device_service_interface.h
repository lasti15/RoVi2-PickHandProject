#ifndef CAROS_CONTROL_SERIAL_DEVICE_SERVICE_INTERFACE_H
#define CAROS_CONTROL_SERIAL_DEVICE_SERVICE_INTERFACE_H

#include <caros_control_msgs/robot_state.h>
#include <caros_control_msgs/serial_device_move_lin.h>
#include <caros_control_msgs/serial_device_move_ptp.h>
#include <caros_control_msgs/serial_device_move_ptp_t.h>
#include <caros_control_msgs/serial_device_move_vel_q.h>
#include <caros_control_msgs/serial_device_move_vel_t.h>
#include <caros_control_msgs/serial_device_move_lin_fc.h>
#include <caros_control_msgs/serial_device_move_servo_q.h>
#include <caros_control_msgs/serial_device_move_servo_t.h>
#include <caros_common_msgs/config_bool.h>
#include <caros_common_msgs/empty_srv.h>

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/Wrench6D.hpp>

#include <ros/ros.h>

#include <string>
#include <tuple>

/* Always publish the latest serial device state */
#define SERIAL_DEVICE_STATE_PUBLISHER_QUEUE_SIZE 1
#define SERIAL_DEVICE_SERVICE_INTERFACE_SUB_NAMESPACE "caros_serial_device_service_interface"

namespace caros
{
/**
 * @brief This is the serial device interface. It defines the (minimum) interface that a joint based robotic device
 *needs to implement.
 *
 * In ROS the namespace of the node is used and it is important that not two GripperServiceInterfaces are running in the
 *same namespace.
 */
class SerialDeviceServiceInterface
{
 public:
  SerialDeviceServiceInterface(ros::NodeHandle nodehandle);

  virtual ~SerialDeviceServiceInterface();

  typedef std::vector<std::tuple<const rw::math::Transform3D<>, const float>> TransformAndSpeedContainer_t;
  typedef std::vector<std::tuple<const rw::math::Q, const float>> QAndSpeedContainer_t;

  /* TODO: Not supporting blends at the moment! */
  //! @brief move robot in a linear Cartesian path
  virtual bool moveLin(const TransformAndSpeedContainer_t& targets) = 0;
  //! @brief move robot from point to point
  virtual bool movePtp(const QAndSpeedContainer_t& targets) = 0;
  //! @brief move robot from point to point but using a pose as target (require invkin)
  virtual bool movePtpT(const TransformAndSpeedContainer_t& targets) = 0;
  //! @brief move robot in a servoing fashion specifying joint velocity targets
  virtual bool moveVelQ(const rw::math::Q& q_vel) = 0;
  //! @brief move robot in a servoing fashion specifying a velocity screw in tool coordinates
  virtual bool moveVelT(const rw::math::VelocityScrew6D<>& t_vel) = 0;
  //! @brief move robot with a hybrid position/force control
  virtual bool moveLinFc(const rw::math::Transform3D<>& posTarget, const rw::math::Transform3D<>& offset,
                         const rw::math::Wrench6D<>& wrenchTarget, const rw::math::Q& controlGain) = 0;

  /**
   * @brief move robot in a servoing fashion specifying a joint configuration
   * @note It is implementation specific whether the targets are being moved to individually, or just the last specified target is chosen. Make sure to look at the specific implementation for the node you are using.
   */
  virtual bool moveServoQ(const QAndSpeedContainer_t& targets) = 0;
  /**
   * @brief move robot in a servoing fashion specifying a pose
   * @note It is implementation specific whether the targets are being moved to individually, or just the last specified target is chosen. Make sure to look at the specific implementation for the node you are using.
   */
  virtual bool moveServoT(const TransformAndSpeedContainer_t& targets) = 0;
  //! @brief start the robot
  virtual bool moveStart() = 0;
  //! @brief hard stop the robot
  virtual bool moveStop() = 0;
  /* movePause should just (temporarily) pause the movement, and allow for it to continue (i.e. the robot should still
   * be able to perform the full trajectory) */
  //! @brief pause the robot, should be able to continue trajectory
  virtual bool movePause() = 0;
  //! @brief enable safe mode, so that robot stops when collisions are detected
  virtual bool moveSetSafeModeEnabled(const bool value) = 0;

 protected:
  /**
   * @brief Initialise this interface, such that the ROS services and publishers become available.
   * @returns a boolean indicating whether all the ROS services and publishers were successfully made available.
   */
  bool configureInterface();

  //! publish robot state
  void publishState(const caros_control_msgs::robot_state& state);

 private:
  /**
   * @brief private default constructor.
   * This is declared as private to enforce deriving classes to call an available public constructor and enforce that
   * the ROS services are properly initialised.
   */
  SerialDeviceServiceInterface();

  bool initService();

  bool moveLinHandle(caros_control_msgs::serial_device_move_lin::Request& request,
                     caros_control_msgs::serial_device_move_lin::Response& response);

  bool movePtpHandle(caros_control_msgs::serial_device_move_ptp::Request& request,
                     caros_control_msgs::serial_device_move_ptp::Response& response);

  bool movePtpTHandle(caros_control_msgs::serial_device_move_ptp_t::Request& request,
                      caros_control_msgs::serial_device_move_ptp_t::Response& response);

  bool moveVelQHandle(caros_control_msgs::serial_device_move_vel_q::Request& request,
                      caros_control_msgs::serial_device_move_vel_q::Response& response);

  bool moveVelTHandle(caros_control_msgs::serial_device_move_vel_t::Request& request,
                      caros_control_msgs::serial_device_move_vel_t::Response& response);

  bool moveServoQHandle(caros_control_msgs::serial_device_move_servo_q::Request& request,
                        caros_control_msgs::serial_device_move_servo_q::Response& response);

  bool moveServoTHandle(caros_control_msgs::serial_device_move_servo_t::Request& request,
                        caros_control_msgs::serial_device_move_servo_t::Response& response);

  bool moveLinFcHandle(caros_control_msgs::serial_device_move_lin_fc::Request& request,
                       caros_control_msgs::serial_device_move_lin_fc::Response& response);

  bool moveStartHandle(caros_common_msgs::empty_srv::Request& request,
                       caros_common_msgs::empty_srv::Response& response);

  bool moveStopHandle(caros_common_msgs::empty_srv::Request& request, caros_common_msgs::empty_srv::Response& response);

  bool movePauseHandle(caros_common_msgs::empty_srv::Request& request,
                       caros_common_msgs::empty_srv::Response& response);

  bool moveSetSafeModeEnabledHandle(caros_common_msgs::config_bool::Request& request,
                                    caros_common_msgs::config_bool::Response& response);

 protected:
  std::string service_name_;
  ros::NodeHandle nodehandle_;

  ros::Publisher deviceStatePublisher_;

  ros::ServiceServer srvMoveLin_;
  ros::ServiceServer srvMovePtp_;
  ros::ServiceServer srvMovePtpT_;
  ros::ServiceServer srvMoveVelQ_;
  ros::ServiceServer srvMoveVelT_;
  ros::ServiceServer srvMoveLinFc_;
  ros::ServiceServer srvMoveServoQ_;
  ros::ServiceServer srvMoveServoT_;

  ros::ServiceServer srvMoveStart_;
  ros::ServiceServer srvMoveStop_;
  ros::ServiceServer srvMovePause_;
  ros::ServiceServer srvSetSafeModeEnabled_;
};
}
#endif  //#ifndef
