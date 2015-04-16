/**/
#include <caros/serial_device_service_interface.h>
#include <caros/common.h>

#include <caros_control_msgs/robot_state.h>

#include <rw/math.hpp>
#include <boost/foreach.hpp>

#include <vector>
#include <tuple>

namespace
{
/* FIXME:
 * Remove hardcoded information regarding the targets and speeds are vectors or (random) indexing data structures  e.g.
 * targets_t::size_t looping and such...
 */
template <typename targets_t, typename speeds_t, typename container_t>
bool fillContainerWithTargetsAndSpeeds(const targets_t& targets, const speeds_t& speeds, container_t& res)
{
  if (targets.size() != speeds.size())
  {
    ROS_WARN_STREAM("There are " << targets.size() << " targets and " << speeds.size()
                                 << " speeds, but there should be the same amount of each!");
    return false;
  }

  ROS_ASSERT(
      targets.size() ==
      speeds
          .size()); /* Just an extra "safety" control to verify that our double indexing loop will behave as expected */
  res.clear();
  res.reserve(targets.size());

  /* TODO:
   * Perform the for-loop within a try-catch block to catch out-of-range access within the .at() call
   */
  for (typename targets_t::size_type index = 0; index < targets.size(); ++index)
  {
    res.push_back(std::make_tuple(caros::toRw(targets.at(index)), speeds.at(index)));
  }

  return true;
}
} // end namespace

using namespace caros;

SerialDeviceServiceInterface::SerialDeviceServiceInterface(ros::NodeHandle nodehandle)
    : nodehandle_(nodehandle, SERIAL_DEVICE_SERVICE_INTERFACE_SUB_NAMESPACE)
{
  /* Do nothing */
}

SerialDeviceServiceInterface::SerialDeviceServiceInterface()
{
  /* Do nothing */
  ROS_FATAL_STREAM(
      "The empty constructor of the GripperServiceInterface should never be called! This is undefined behaviour.");
}

SerialDeviceServiceInterface::~SerialDeviceServiceInterface()
{
  /* Nothing special needs to be done - relying on ROS's RAII design */
}

bool SerialDeviceServiceInterface::configureInterface()
{
  return initService();
}

bool SerialDeviceServiceInterface::initService()
{
  if (srvMoveLin_ || srvMovePtp_ || srvMovePtpT_ || srvMoveVelQ_ || srvMoveVelT_ || srvMoveServoQ_ || srvMoveServoT_ ||
      srvMoveLinFc_ || srvMoveStart_ || srvMoveStop_ || srvMovePause_ || srvSetSafeModeEnabled_ ||
      deviceStatePublisher_)
  {
    ROS_WARN_STREAM(
        "Reinitialising one or more SerialDeviceServiceInterface services or publishers. If this is not fully intended "
        "then this should be considered a bug!");
  }

  /* TODO:
   * Should the "RobotState" not be called something like SerialDeviceState or similar?
   * ^- The name should also be replaced in the ros error stream condition statement!
   */
  deviceStatePublisher_ =
      nodehandle_.advertise<caros_control_msgs::robot_state>("robot_state", SERIAL_DEVICE_STATE_PUBLISHER_QUEUE_SIZE);
  ROS_ERROR_STREAM_COND(!deviceStatePublisher_, "The RobotState publisher is empty!");

  srvMoveLin_ = nodehandle_.advertiseService("move_lin", &SerialDeviceServiceInterface::moveLinHandle, this);
  ROS_ERROR_STREAM_COND(!srvMoveLin_, "The move_lin service is empty!");

  srvMovePtp_ = nodehandle_.advertiseService("move_ptp", &SerialDeviceServiceInterface::movePtpHandle, this);
  ROS_ERROR_STREAM_COND(!srvMovePtp_, "The move_ptp service is empty!");

  srvMovePtpT_ = nodehandle_.advertiseService("move_ptp_t", &SerialDeviceServiceInterface::movePtpTHandle, this);
  ROS_ERROR_STREAM_COND(!srvMovePtpT_, "The move_ptp_t service is empty!");

  srvMoveVelQ_ = nodehandle_.advertiseService("move_vel_q", &SerialDeviceServiceInterface::moveVelQHandle, this);
  ROS_ERROR_STREAM_COND(!srvMoveVelQ_, "The move_vel_q service is empty!");

  srvMoveVelT_ = nodehandle_.advertiseService("move_vel_t", &SerialDeviceServiceInterface::moveVelTHandle, this);
  ROS_ERROR_STREAM_COND(!srvMoveVelT_, "The move_vel_t service is empty!");

  srvMoveServoQ_ = nodehandle_.advertiseService("move_servo_q", &SerialDeviceServiceInterface::moveServoQHandle, this);
  ROS_ERROR_STREAM_COND(!srvMoveServoQ_, "The move_servo_q service is empty!");

  srvMoveServoT_ = nodehandle_.advertiseService("move_servo_t", &SerialDeviceServiceInterface::moveServoTHandle, this);
  ROS_ERROR_STREAM_COND(!srvMoveServoT_, "The move_servo_t service is empty!");

  srvMoveLinFc_ = nodehandle_.advertiseService("move_lin_fc", &SerialDeviceServiceInterface::moveLinFcHandle, this);
  ROS_ERROR_STREAM_COND(!srvMoveLinFc_, "The move_lin_fc service is empty!");

  srvMoveStart_ = nodehandle_.advertiseService("move_start", &SerialDeviceServiceInterface::moveStartHandle, this);
  ROS_ERROR_STREAM_COND(!srvMoveStart_, "The move_start service is empty!");

  srvMoveStop_ = nodehandle_.advertiseService("move_stop", &SerialDeviceServiceInterface::moveStopHandle, this);
  ROS_ERROR_STREAM_COND(!srvMoveStop_, "The move_stop service is empty!");

  srvMovePause_ = nodehandle_.advertiseService("move_pause", &SerialDeviceServiceInterface::movePauseHandle, this);
  ROS_ERROR_STREAM_COND(!srvMovePause_, "The move_pause service is empty!");

  srvSetSafeModeEnabled_ = nodehandle_.advertiseService(
      "set_safe_mode_enabled", &SerialDeviceServiceInterface::moveSetSafeModeEnabledHandle, this);
  ROS_ERROR_STREAM_COND(!srvSetSafeModeEnabled_, "The set_safe_mode_enabled service is empty!");

  if (srvMoveLin_ && srvMovePtp_ && srvMovePtpT_ && srvMoveVelQ_ && srvMoveVelT_ && srvMoveServoQ_ && srvMoveServoT_ &&
      srvMoveLinFc_ && srvMoveStart_ && srvMoveStop_ && srvMovePause_ && srvSetSafeModeEnabled_ &&
      deviceStatePublisher_)
  {
    /* Everything seems to be properly initialised */
    ROS_DEBUG_STREAM(
        "All SerialDeviceServiceInterface publishers and services appear to have been properly initialised");
  }
  else
  {
    ROS_ERROR_STREAM(
        "The SerialDeviceServiceInterface could not be properly initialised - one or more ROS services or publishers "
        "failed to be properly initialised.");
    return false;
  }

  return true;
}

void SerialDeviceServiceInterface::publish(const caros_control_msgs::robot_state& state)
{
  deviceStatePublisher_.publish(state);
}

/************************************************************************
 * ROS service handle functions
 ************************************************************************/
/* TODO:
 * Rewrite the functions to also take in the blends (which should be added to the .srv files). Also remember to update
 * the non-handle versions.
 * Try to refactor as much of the shared conversion/code for all these methods, into separate functions, like with the
 * fillContainerWithTransformsAndSpeed(...) function.
 */
bool SerialDeviceServiceInterface::moveLinHandle(caros_control_msgs::serial_device_move_lin::Request& request,
                                                 caros_control_msgs::serial_device_move_lin::Response& response)
{
  TransformAndSpeedContainer_t res;
  if (fillContainerWithTargetsAndSpeeds(request.targets, request.speeds, res))
  {
    response.success = moveLin(res);
  }
  else
  {
    response.success = false;
  }

  return true;
}

bool SerialDeviceServiceInterface::movePtpHandle(caros_control_msgs::serial_device_move_ptp::Request& request,
                                                 caros_control_msgs::serial_device_move_ptp::Response& response)
{
  QAndSpeedContainer_t res;
  if (fillContainerWithTargetsAndSpeeds(request.targets, request.speeds, res))
  {
    response.success = movePtp(res);
  }
  else
  {
    response.success = false;
  }

  return true;
}

bool SerialDeviceServiceInterface::movePtpTHandle(caros_control_msgs::serial_device_move_ptp_t::Request& request,
                                                   caros_control_msgs::serial_device_move_ptp_t::Response& response)
{
  TransformAndSpeedContainer_t res;
  if (fillContainerWithTargetsAndSpeeds(request.targets, request.speeds, res))
  {
    response.success = movePtpT(res);
  }
  else
  {
    response.success = false;
  }

  return true;
}

bool SerialDeviceServiceInterface::moveVelQHandle(caros_control_msgs::serial_device_move_vel_q::Request& request,
                                                  caros_control_msgs::serial_device_move_vel_q::Response& response)
{
  rw::math::Q vel = caros::toRw(request.vel);
  response.success = moveVelQ(vel);

  return true;
}

bool SerialDeviceServiceInterface::moveVelTHandle(caros_control_msgs::serial_device_move_vel_t::Request& request,
                                                  caros_control_msgs::serial_device_move_vel_t::Response& response)
{
  rw::math::VelocityScrew6D<> vel = caros::toRw(request.vel);
  response.success = moveVelT(vel);

  return true;
}

/* TODO:
 * Add speeds implementation
 * This should also be added to the RobWorkHardware URCallBackInterface servo functioninality, where the speed should be
 * optional, defaulting to the currently hardcoded value...
 */
bool SerialDeviceServiceInterface::moveServoQHandle(caros_control_msgs::serial_device_move_servo_q::Request& request,
                                                    caros_control_msgs::serial_device_move_servo_q::Response& response)
{
  QAndSpeedContainer_t res;
  if (fillContainerWithTargetsAndSpeeds(request.targets, request.speeds, res))
  {
    response.success = moveServoQ(res);
  }
  else
  {
    response.success = false;
  }

  return true;
}

bool SerialDeviceServiceInterface::moveServoTHandle(caros_control_msgs::serial_device_move_servo_t::Request& request,
                                                    caros_control_msgs::serial_device_move_servo_t::Response& response)
{
  TransformAndSpeedContainer_t res;
  if (fillContainerWithTargetsAndSpeeds(request.targets, request.speeds, res))
  {
    response.success = moveServoT(res);
  }
  else
  {
    response.success = false;
  }

  return true;
}

bool SerialDeviceServiceInterface::moveLinFcHandle(caros_control_msgs::serial_device_move_lin_fc::Request& request,
                                                   caros_control_msgs::serial_device_move_lin_fc::Response& response)
{
  rw::math::Transform3D<> posTarget = caros::toRw(request.pos_target);
  rw::math::Transform3D<> offset = caros::toRw(request.offset);

  rw::math::Wrench6D<> wrenchTarget;
  wrenchTarget(0) = request.wrench_target.force.x;
  wrenchTarget(1) = request.wrench_target.force.y;
  wrenchTarget(2) = request.wrench_target.force.z;

  wrenchTarget(3) = request.wrench_target.torque.x;
  wrenchTarget(4) = request.wrench_target.torque.y;
  wrenchTarget(5) = request.wrench_target.torque.z;

  /* FIXME:
   * Add proper output message!
   */
  //    ROS_ASSERT_MSG(request.ctrl_gains.size() == 6, "The size of ctrl_gains " << request.ctrl_gains.size() << " but
  //    should be 6!");
  ROS_ASSERT(request.ctrl_gains.size() == 6);
  /* FIXME:
   * Convert the ctrl_gains (boost::array) to std::vector or rw::math::Q ?
   */
  rw::math::Q selection(request.ctrl_gains.size(), request.ctrl_gains.at(0), request.ctrl_gains.at(1),
                        request.ctrl_gains.at(2), request.ctrl_gains.at(3), request.ctrl_gains.at(4),
                        request.ctrl_gains.at(5));

  response.success = moveLinFc(posTarget, offset, wrenchTarget, selection);

  return true;
}

bool SerialDeviceServiceInterface::moveStartHandle(caros_common_msgs::empty_srv::Request& request,
                                                   caros_common_msgs::empty_srv::Response& response)
{
  response.success = moveStart();

  return true;
}

bool SerialDeviceServiceInterface::moveStopHandle(caros_common_msgs::empty_srv::Request& request,
                                                  caros_common_msgs::empty_srv::Response& response)
{
  response.success = moveStop();

  return true;
}

bool SerialDeviceServiceInterface::movePauseHandle(caros_common_msgs::empty_srv::Request& request,
                                                   caros_common_msgs::empty_srv::Response& response)
{
  response.success = movePause();

  return true;
}

bool SerialDeviceServiceInterface::moveSetSafeModeEnabledHandle(caros_common_msgs::config_bool::Request& request,
                                                                caros_common_msgs::config_bool::Response& response)
{
  bool value = request.value;
  response.success = moveSetSafeModeEnabled(value);

  return true;
}
