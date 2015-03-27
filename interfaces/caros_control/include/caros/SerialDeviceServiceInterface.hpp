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

/* TODO:
 * The publisher queue size was specified to be 10, but why 10 and not just the latest?
 */
#define SERIAL_DEVICE_STATE_PUBLISHER_QUEUE_SIZE 10
#define SERIAL_DEVICE_SERVICE_INTERFACE_SUB_NAMESPACE "caros_serial_device_service_interface"

namespace caros
{
/* TODO:
 * Should this interface also contain a cleanupInterface (or private cleanupService() and public deconfigureInterface()
 * or shutdownInterface()) ?
 * ^- The GripperServiceInterface contains these sort of functions
 * ^^-- But the interfaces could also just be converted to RAII style and have them enabled by default without the
 * option to switch them off - ros::spin[all] (or similar) has to be called anyway for the processing of the service
 * requests, so it's not like the services can be called before the node is ready to process them anyway.
 */

/**
 * @brief this is the top level robot arm interface. It defines the
 * minimum interface that a joint based robotic arm device needs
 * to implement.
 *
 * The namespace of the nodehandle/service_name is used and it is important that
 * not two RobotArmServiceInterfaces are running in the same namespace.
 */
class SerialDeviceServiceInterface
{
 public:
  SerialDeviceServiceInterface(ros::NodeHandle nodehandle);

  virtual ~SerialDeviceServiceInterface();

  typedef std::vector<std::tuple<const rw::math::Transform3D<>, const float>> TransformAndSpeedContainer_t;
  typedef std::vector<std::tuple<const rw::math::Q, const float>> QAndSpeedContainer_t;

  /* FIXME:
   * Streamline the .srv descriptions and comments to have the same wording (especially for the speeds part)
   * ^- Also look at the return/response fields and make sure they are streamlined (currently they should only return a
   * boolean based on the success of the ROS service call (not whether the actually action has been done or not - just
   * that it was a valid action (i.e. same amount of targets and speeds, and that IK solutions could be found).
   */
  /* Not supporting blends at the moment! */
  /* FIXME:
   * Add API documentation for these functions (see the "old" version of this file for hints)
   */
  //! @brief move robot in a linear Cartesian path
  virtual bool moveLin(const TransformAndSpeedContainer_t& targets) = 0;
  //! @brief move robot from point to point
  virtual bool movePTP(const QAndSpeedContainer_t& targets) = 0;
  //! @brief move robot from point to point but using a pose as target (require invkin)
  virtual bool movePTP_T(const TransformAndSpeedContainer_t& targets) = 0;
  //! @brief move robot in a servoing fasion specifying joint velocity targets
  virtual bool moveVelQ(const rw::math::Q& q_vel) = 0;
  //! @brief move robot in a servoing fasion specifying a velocity screw in tool coordinates
  virtual bool moveVelT(const rw::math::VelocityScrew6D<>& t_vel) = 0;
  //! @brief move robot with a hybrid position/force control
  virtual bool moveLinFC(const rw::math::Transform3D<>& posTarget, const rw::math::Transform3D<>& offset,
                         const rw::math::Wrench6D<>& wrenchTarget, const rw::math::Q& controlGain) = 0;

  /* TODO:
   * Does it make sense that the servoQ and servoT are taking in multiple targets, instead of 1 target for each
   * invocation?
   * As far as I understand it, then servoing is supposed to just move to the newest configuration and throw away all
   * the intermediate configurations as they are irrelevant and won't be used as waypoints.
   */
  //! @brief move robot in a servoing fasion specifying a joint configuration
  virtual bool moveServoQ(const QAndSpeedContainer_t& targets) = 0;
  //! @brief move robot in a servoing fasion specifying a pose
  virtual bool moveServoT(const TransformAndSpeedContainer_t& targets) = 0;
  //! @brief start the robot
  virtual bool moveStart() = 0;
  //! @brief hard stop the robot
  virtual bool moveStop() = 0;
  /* movePause should just (temporarily) pause the movement, and allow for it to continue (i.e. the robot should still
   * be able to perform the full trajectory) */
  //! @brief pause the robot, should be able to continue trajectory
  virtual bool movePause() = 0;
  /* TODO:
   * Is there a better name or multiple methods e.g. enable and disable for the following functionality?
   * Also make sure to properly setup the arguments according to the above decisions
   */
  //! @brief enable safe mode, so that robot stops when collisions are detected
  virtual bool moveSetSafeModeEnabled(const bool value) = 0;

 protected:
  /* FIXME: add api documentation */
  bool configureInterface();

  //! publish robot state
  void publish(const caros_control_msgs::robot_state& state);

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

  bool movePTPHandle(caros_control_msgs::serial_device_move_ptp::Request& request,
                     caros_control_msgs::serial_device_move_ptp::Response& response);

  bool movePTP_THandle(caros_control_msgs::serial_device_move_ptp_t::Request& request,
                       caros_control_msgs::serial_device_move_ptp_t::Response& response);

  bool moveVelQHandle(caros_control_msgs::serial_device_move_vel_q::Request& request,
                      caros_control_msgs::serial_device_move_vel_q::Response& response);

  bool moveVelTHandle(caros_control_msgs::serial_device_move_vel_t::Request& request,
                      caros_control_msgs::serial_device_move_vel_t::Response& response);

  bool moveServoQHandle(caros_control_msgs::serial_device_move_servo_q::Request& request,
                        caros_control_msgs::serial_device_move_servo_q::Response& response);

  bool moveServoTHandle(caros_control_msgs::serial_device_move_servo_t::Request& request,
                        caros_control_msgs::serial_device_move_servo_t::Response& response);

  bool moveLinFCHandle(caros_control_msgs::serial_device_move_lin_fc::Request& request,
                       caros_control_msgs::serial_device_move_lin_fc::Response& response);

  bool moveStartHandle(caros_common_msgs::empty_srv::Request& request,
                       caros_common_msgs::empty_srv::Response& response);

  bool moveStopHandle(caros_common_msgs::empty_srv::Request& request, caros_common_msgs::empty_srv::Response& response);

  bool movePauseHandle(caros_common_msgs::empty_srv::Request& request,
                       caros_common_msgs::empty_srv::Response& response);

  bool moveSetSafeModeEnabledHandle(caros_common_msgs::config_bool::Request& request,
                                    caros_common_msgs::config_bool::Response& response);

 protected:
  std::string _service_name;
  ros::NodeHandle _nodehandle;

  ros::Publisher _deviceStatePublisher;

  ros::ServiceServer _srvMoveLin;
  ros::ServiceServer _srvMovePTP;
  ros::ServiceServer _srvMovePTP_T;
  ros::ServiceServer _srvMoveVelQ;
  ros::ServiceServer _srvMoveVelT;
  ros::ServiceServer _srvMoveLinFC;
  ros::ServiceServer _srvMoveServoQ;
  ros::ServiceServer _srvMoveServoT;

  ros::ServiceServer _srvMoveStart;
  ros::ServiceServer _srvMoveStop;
  ros::ServiceServer _srvMovePause;
  ros::ServiceServer _srvSetSafeModeEnabled;
};
}
#endif  //#ifndef
