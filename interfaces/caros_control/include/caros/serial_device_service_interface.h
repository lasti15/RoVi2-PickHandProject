#ifndef CAROS_SERIAL_DEVICE_SERVICE_INTERFACE_H
#define CAROS_SERIAL_DEVICE_SERVICE_INTERFACE_H

#include <caros_control_msgs/RobotState.h>
#include <caros_control_msgs/SerialDeviceMoveLin.h>
#include <caros_control_msgs/SerialDeviceMovePtp.h>
#include <caros_control_msgs/SerialDeviceMovePtpT.h>
#include <caros_control_msgs/SerialDeviceMoveVelQ.h>
#include <caros_control_msgs/SerialDeviceMoveVelT.h>
#include <caros_control_msgs/SerialDeviceMoveLinFc.h>
#include <caros_control_msgs/SerialDeviceMoveServoQ.h>
#include <caros_control_msgs/SerialDeviceMoveServoT.h>
#include <caros_common_msgs/ConfigBool.h>
#include <caros_common_msgs/EmptySrv.h>

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
  explicit SerialDeviceServiceInterface(ros::NodeHandle nodehandle);

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
  virtual bool moveLinFc(const rw::math::Transform3D<>& pos_target, const rw::math::Transform3D<>& offset,
                         const rw::math::Wrench6D<>& wrench_target, const rw::math::Q& control_gain) = 0;

  /**
   * @brief move robot in a servoing fashion specifying a joint configuration
   * @note It is implementation specific whether the targets are being moved to individually, or just the last specified
   * target is chosen. Make sure to look at the specific implementation for the node you are using.
   */
  virtual bool moveServoQ(const QAndSpeedContainer_t& targets) = 0;
  /**
   * @brief move robot in a servoing fashion specifying a pose
   * @note It is implementation specific whether the targets are being moved to individually, or just the last specified
   * target is chosen. Make sure to look at the specific implementation for the node you are using.
   */
  virtual bool moveServoT(const TransformAndSpeedContainer_t& targets) = 0;
  //! @brief hard stop the robot
  virtual bool moveStop() = 0;
  //! @brief enable safe mode, so that robot stops when collisions are detected
  virtual bool moveSetSafeModeEnabled(const bool value) = 0;

 protected:
  /**
   * @brief Initialise this interface, such that the ROS services and publishers become available.
   * @returns a boolean indicating whether all the ROS services and publishers were successfully made available.
   */
  bool configureInterface();

  //! publish robot state
  void publishState(const caros_control_msgs::RobotState& state);

 private:
  /**
   * @brief private default constructor.
   * This is declared as private to enforce deriving classes to call an available public constructor and enforce that
   * the ROS services are properly initialised.
   */
  SerialDeviceServiceInterface();

  bool initService();

  bool moveLinHandle(caros_control_msgs::SerialDeviceMoveLin::Request& request,
                     caros_control_msgs::SerialDeviceMoveLin::Response& response);

  bool movePtpHandle(caros_control_msgs::SerialDeviceMovePtp::Request& request,
                     caros_control_msgs::SerialDeviceMovePtp::Response& response);

  bool movePtpTHandle(caros_control_msgs::SerialDeviceMovePtpT::Request& request,
                      caros_control_msgs::SerialDeviceMovePtpT::Response& response);

  bool moveVelQHandle(caros_control_msgs::SerialDeviceMoveVelQ::Request& request,
                      caros_control_msgs::SerialDeviceMoveVelQ::Response& response);

  bool moveVelTHandle(caros_control_msgs::SerialDeviceMoveVelT::Request& request,
                      caros_control_msgs::SerialDeviceMoveVelT::Response& response);

  bool moveServoQHandle(caros_control_msgs::SerialDeviceMoveServoQ::Request& request,
                        caros_control_msgs::SerialDeviceMoveServoQ::Response& response);

  bool moveServoTHandle(caros_control_msgs::SerialDeviceMoveServoT::Request& request,
                        caros_control_msgs::SerialDeviceMoveServoT::Response& response);

  bool moveLinFcHandle(caros_control_msgs::SerialDeviceMoveLinFc::Request& request,
                       caros_control_msgs::SerialDeviceMoveLinFc::Response& response);

  bool moveStopHandle(caros_common_msgs::EmptySrv::Request& request, caros_common_msgs::EmptySrv::Response& response);

  bool moveSetSafeModeEnabledHandle(caros_common_msgs::ConfigBool::Request& request,
                                    caros_common_msgs::ConfigBool::Response& response);

 protected:
  std::string service_name_;
  ros::NodeHandle nodehandle_;

  ros::Publisher device_state_publisher_;

  ros::ServiceServer srv_move_lin_;
  ros::ServiceServer srv_move_ptp_;
  ros::ServiceServer srv_move_ptp_t_;
  ros::ServiceServer srv_move_vel_q_;
  ros::ServiceServer srv_move_vel_t_;
  ros::ServiceServer srv_move_lin_fc_;
  ros::ServiceServer srv_move_servo_q_;
  ros::ServiceServer srv_move_servo_t_;

  ros::ServiceServer srv_move_stop_;
  ros::ServiceServer srv_set_safe_mode_enabled_;
};
}  // namespace caros

#endif  // CAROS_SERIAL_DEVICE_SERVICE_INTERFACE_H
