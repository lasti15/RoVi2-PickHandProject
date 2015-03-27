#include <caros/serial_device_si_proxy.h>

#include <caros/serial_device_service_interface.h>
#include <caros/common.h>
#include <caros_common_msgs/stop.h>
#include <caros_common_msgs/pause.h>
#include <caros_common_msgs/config_bool.h>
#include <caros_control_msgs/serial_device_move_lin.h>
#include <caros_control_msgs/serial_device_move_ptp.h>
#include <caros_control_msgs/serial_device_move_ptp_t.h>
#include <caros_control_msgs/serial_device_move_vel_q.h>
#include <caros_control_msgs/serial_device_move_vel_t.h>
#include <caros_control_msgs/serial_device_move_lin_fc.h>
#include <caros_control_msgs/serial_device_move_servo_q.h>
#include <caros_control_msgs/serial_device_move_servo_t.h>

#include <sstream>
#include <cmath>

#define SPEED_MIN 0.0f
#define SPEED_MAX 100.0f

namespace
{
/* If doing this as a do {...} while(0) macro, then there wouldn't be any type enforcement */
/* Throw an appropriate exception if the value is not within [min;max] */
void verifyValueIsWithin(const float& value, const float& min, const float& max)
{
  ROS_DEBUG_STREAM("Verifying that the value '" << value << "' is within [" << min << ";" << max << "]");
  if (std::isnan(min) || std::isnan(max))
  {
    throw std::invalid_argument("Make sure both min and max are not NaN's");
  }
  else if (std::isnan(value))
  {
    throw std::invalid_argument("The value is considered NaN");
  }
  else if (not(std::isgreaterequal(value, min) && std::islessequal(value, max)))
  {
    std::ostringstream oss;
    oss << "The value is not within [" << min << ";" << max << "]";
    throw std::range_error(oss.str());
  }
}
}  // end namespace

using namespace caros;

SerialDeviceSIProxy::SerialDeviceSIProxy(ros::NodeHandle nodehandle, const std::string& devname)
    : nodehandle_(nodehandle)
{
  std::ostringstream rosNamespace;
  rosNamespace << "/" << devname << "/" << SERIAL_DEVICE_SERVICE_INTERFACE_SUB_NAMESPACE;

  /* TODO:
   * Consider persistent connections - especially for the servoQ - or make it a configurable parameter/argument
   */
  srvMoveLin_ = nodehandle_.serviceClient<caros_control_msgs::serial_device_move_lin>(rosNamespace.str() + "/move_lin");
  srvMovePTP_ = nodehandle_.serviceClient<caros_control_msgs::serial_device_move_ptp>(rosNamespace.str() + "/move_ptp");
  srvMovePTP_T_ =
      nodehandle_.serviceClient<caros_control_msgs::serial_device_move_ptp_t>(rosNamespace.str() + "/move_ptp_t");
  srvMoveVelQ_ =
      nodehandle_.serviceClient<caros_control_msgs::serial_device_move_vel_q>(rosNamespace.str() + "/move_vel_q");
  srvMoveVelT_ =
      nodehandle_.serviceClient<caros_control_msgs::serial_device_move_vel_t>(rosNamespace.str() + "/move_vel_t");
  srvMoveServoQ_ =
      nodehandle_.serviceClient<caros_control_msgs::serial_device_move_servo_q>(rosNamespace.str() + "/move_servo_q");
  srvMoveServoT_ =
      nodehandle_.serviceClient<caros_control_msgs::serial_device_move_servo_t>(rosNamespace.str() + "/move_servo_t");
  srvMoveLinFC_ =
      nodehandle_.serviceClient<caros_control_msgs::serial_device_move_lin_fc>(rosNamespace.str() + "/move_lin_fc");
  srvStart_ = nodehandle_.serviceClient<caros_common_msgs::empty_srv>(rosNamespace.str() + "/move_start");
  srvStop_ = nodehandle_.serviceClient<caros_common_msgs::empty_srv>(rosNamespace.str() + "/move_stop");
  srvPause_ = nodehandle_.serviceClient<caros_common_msgs::empty_srv>(rosNamespace.str() + "/move_pause");
  srvSetSafeModeEnabled_ =
      nodehandle_.serviceClient<caros_common_msgs::config_bool>(rosNamespace.str() + "/set_safe_mode_enabled");

  // states
  subRobotState_ =
      nodehandle_.subscribe(rosNamespace.str() + "/robot_state", 1, &SerialDeviceSIProxy::handleRobotState, this);
}

SerialDeviceSIProxy::~SerialDeviceSIProxy()
{
}

bool SerialDeviceSIProxy::moveLin(const rw::math::Transform3D<>& target, const float speed, const float blend)
{
  bool srvCallSuccess = false;
  verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
  caros_control_msgs::serial_device_move_lin srv;
  srv.request.targets.push_back(caros::toRos(target));
  srv.request.speeds.push_back(caros::toRos(speed));
  srv.request.blends.push_back(caros::toRos(blend));

  if (not srvMoveLin_.exists())
  {
    THROW_CAROS_UNAVAILABLE_SERVICE("The service " << srvMoveLin_.getService() << " does not exist.");
  }

  srvCallSuccess = srvMoveLin_.call(srv);
  if (not srvCallSuccess)
  {
    THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << srvMoveLin_.getService());
  }

  return srv.response.success;
}

bool SerialDeviceSIProxy::movePTP(const rw::math::Q& target, const float speed, const float blend)
{
  bool srvCallSuccess = false;
  verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
  caros_control_msgs::serial_device_move_ptp srv;
  srv.request.targets.push_back(caros::toRos(target));
  srv.request.speeds.push_back(caros::toRos(speed));
  srv.request.blends.push_back(caros::toRos(blend));

  if (not srvMovePTP_.exists())
  {
    THROW_CAROS_UNAVAILABLE_SERVICE("The service " << srvMovePTP_.getService() << " does not exist.");
  }

  srvCallSuccess = srvMovePTP_.call(srv);
  if (not srvCallSuccess)
  {
    THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << srvMovePTP_.getService());
  }

  return srv.response.success;
}

bool SerialDeviceSIProxy::movePTP_T(const rw::math::Transform3D<>& target, const float speed, const float blend)
{
  bool srvCallSuccess = false;
  verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
  caros_control_msgs::serial_device_move_ptp_t srv;
  srv.request.targets.push_back(caros::toRos(target));
  srv.request.speeds.push_back(caros::toRos(speed));
  srv.request.blends.push_back(caros::toRos(blend));

  if (not srvMovePTP_T_.exists())
  {
    THROW_CAROS_UNAVAILABLE_SERVICE("The service " << srvMovePTP_T_.getService() << " does not exist.");
  }

  srvCallSuccess = srvMovePTP_T_.call(srv);
  if (not srvCallSuccess)
  {
    THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << srvMovePTP_T_.getService());
  }

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveServoQ(const rw::math::Q& target, const float speed)
{
  bool srvCallSuccess = false;
  verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
  caros_control_msgs::serial_device_move_servo_q srv;
  srv.request.targets.push_back(caros::toRos(target));
  srv.request.speeds.push_back(caros::toRos(speed));

  if (not srvMoveServoQ_.exists())
  {
    THROW_CAROS_UNAVAILABLE_SERVICE("The service " << srvMoveServoQ_.getService() << " does not exist.");
  }

  srvCallSuccess = srvMoveServoQ_.call(srv);
  if (not srvCallSuccess)
  {
    THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << srvMoveServoQ_.getService());
  }

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveServoT(const rw::math::Transform3D<>& target, const float speed)
{
  bool srvCallSuccess = false;
  verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
  caros_control_msgs::serial_device_move_servo_t srv;
  srv.request.targets.push_back(caros::toRos(target));
  srv.request.speeds.push_back(caros::toRos(speed));

  if (not srvMoveServoT_.exists())
  {
    THROW_CAROS_UNAVAILABLE_SERVICE("The service " << srvMoveServoT_.getService() << " does not exist.");
  }

  srvCallSuccess = srvMoveServoT_.call(srv);
  if (not srvCallSuccess)
  {
    THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << srvMoveServoT_.getService());
  }

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveVelQ(const rw::math::Q& target)
{
  bool srvCallSuccess = false;
  caros_control_msgs::serial_device_move_vel_q srv;
  srv.request.vel = caros::toRos(target);

  if (not srvMoveVelQ_.exists())
  {
    THROW_CAROS_UNAVAILABLE_SERVICE("The service " << srvMoveVelQ_.getService() << " does not exist.");
  }

  srvCallSuccess = srvMoveVelQ_.call(srv);
  if (not srvCallSuccess)
  {
    THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << srvMoveVelQ_.getService());
  }

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveVelT(const rw::math::VelocityScrew6D<>& target)
{
  bool srvCallSuccess = false;
  caros_control_msgs::serial_device_move_vel_t srv;
  srv.request.vel = caros::toRos(target);

  if (not srvMoveVelT_.exists())
  {
    THROW_CAROS_UNAVAILABLE_SERVICE("The service " << srvMoveVelT_.getService() << " does not exist.");
  }

  srvCallSuccess = srvMoveVelT_.call(srv);
  if (not srvCallSuccess)
  {
    THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << srvMoveVelT_.getService());
  }

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveLinFC(const rw::math::Transform3D<>& posTarget, const rw::math::Transform3D<>& offset,
                                    const rw::math::Wrench6D<>& wrenchTarget, const rw::math::Q& controlGain)
{
  bool srvCallSuccess = false;
  caros_control_msgs::serial_device_move_lin_fc srv;
  srv.request.pos_target = caros::toRos(posTarget);
  srv.request.offset = caros::toRos(offset);
  srv.request.wrench_target = caros::toRos(wrenchTarget);

  ROS_ASSERT(controlGain.size() == srv.request.ctrl_gains.size());
  for (std::size_t index = 0; index < srv.request.ctrl_gains.size(); ++index)
  {
    srv.request.ctrl_gains[index] = caros::toRosFloat(controlGain[index]);
  }

  if (not srvMoveLinFC_.exists())
  {
    THROW_CAROS_UNAVAILABLE_SERVICE("The service " << srvMoveLinFC_.getService() << " does not exist.");
  }

  srvCallSuccess = srvMoveLinFC_.call(srv);
  if (not srvCallSuccess)
  {
    THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << srvMoveLinFC_.getService());
  }

  return srv.response.success;
}

bool SerialDeviceSIProxy::stop()
{
  bool srvCallSuccess = false;
  caros_common_msgs::stop srv;

  if (not srvStop_.exists())
  {
    THROW_CAROS_UNAVAILABLE_SERVICE("The service " << srvStop_.getService() << " does not exist.");
  }

  srvCallSuccess = srvStop_.call(srv);
  if (not srvCallSuccess)
  {
    THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << srvStop_.getService());
  }

  return srv.response.success;
}

bool SerialDeviceSIProxy::pause()
{
  bool srvCallSuccess = false;
  caros_common_msgs::pause srv;

  if (not srvPause_.exists())
  {
    THROW_CAROS_UNAVAILABLE_SERVICE("The service " << srvPause_.getService() << " does not exist.");
  }

  srvCallSuccess = srvPause_.call(srv);
  if (not srvCallSuccess)
  {
    THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << srvPause_.getService());
  }

  return srv.response.success;
}

bool SerialDeviceSIProxy::setSafeModeEnabled(bool enable)
{
  bool srvCallSuccess = false;
  caros_common_msgs::config_bool srv;
  srv.request.value = caros::toRos(enable);

  if (not srvSetSafeModeEnabled_.exists())
  {
    THROW_CAROS_UNAVAILABLE_SERVICE("The service " << srvSetSafeModeEnabled_.getService() << " does not exist.");
  }

  srvCallSuccess = srvSetSafeModeEnabled_.call(srv);
  if (not srvCallSuccess)
  {
    THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << srvSetSafeModeEnabled_.getService());
  }

  return srv.response.success;
}

void SerialDeviceSIProxy::handleRobotState(const caros_control_msgs::robot_state& state)
{
  pRobotState_ = state;
}

rw::math::Q SerialDeviceSIProxy::getQ()
{
  return caros::toRw(pRobotState_.q);
}

rw::math::Q SerialDeviceSIProxy::getQd()
{
  return caros::toRw(pRobotState_.dq);
}

bool SerialDeviceSIProxy::isMoving()
{
  return pRobotState_.isMoving;
}

ros::Time SerialDeviceSIProxy::getTimeStamp()
{
  return pRobotState_.header.stamp;
}
