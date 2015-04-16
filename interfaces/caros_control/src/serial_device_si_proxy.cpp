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

#define SPEED_MIN 0.0f
#define SPEED_MAX 100.0f

using namespace caros;

SerialDeviceSIProxy::SerialDeviceSIProxy(ros::NodeHandle nodehandle, const std::string& devname,
                                         const bool usePersistentConnections)
    : nodehandle_(nodehandle),
      usePersistentConnections_(usePersistentConnections),
      rosNamespace_("/" + devname + "/" + SERIAL_DEVICE_SERVICE_INTERFACE_SUB_NAMESPACE),
      srvMoveLinFc_(nodehandle_, "move_lin_fc", rosNamespace_, usePersistentConnections_),
      srvMoveLin_(nodehandle_, "move_lin", rosNamespace_, usePersistentConnections_),
      srvMovePtp_(nodehandle_, "move_ptp", rosNamespace_, usePersistentConnections_),
      srvMovePtpT_(nodehandle_, "move_ptp_t", rosNamespace_, usePersistentConnections_),
      srvMoveServoQ_(nodehandle_, "move_servo_q", rosNamespace_, usePersistentConnections_),
      srvMoveServoT_(nodehandle_, "move_servo_t", rosNamespace_, usePersistentConnections_),
      srvMoveVelQ_(nodehandle_, "move_vel_q", rosNamespace_, usePersistentConnections_),
      srvMoveVelT_(nodehandle_, "move_vel_t", rosNamespace_, usePersistentConnections_),
      srvPause_(nodehandle_, "move_pause", rosNamespace_, usePersistentConnections_),
      srvStart_(nodehandle_, "move_start", rosNamespace_, usePersistentConnections_),
      srvStop_(nodehandle_, "move_stop", rosNamespace_, usePersistentConnections_),
      srvSetSafeModeEnabled_(nodehandle_, "set_safe_mode_enabled", rosNamespace_, usePersistentConnections_)
{
  // states
  subRobotState_ =
      nodehandle_.subscribe(rosNamespace_ + "/robot_state", 1, &SerialDeviceSIProxy::handleRobotState, this);
}

SerialDeviceSIProxy::~SerialDeviceSIProxy()
{
}

bool SerialDeviceSIProxy::moveLin(const rw::math::Transform3D<>& target, const float speed, const float blend)
{
  verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
  caros_control_msgs::serial_device_move_lin srv;
  srv.request.targets.push_back(caros::toRos(target));
  srv.request.speeds.push_back(caros::toRos(speed));
  srv.request.blends.push_back(caros::toRos(blend));

  srvMoveLin_.call<caros_control_msgs::serial_device_move_lin>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::movePtp(const rw::math::Q& target, const float speed, const float blend)
{
  verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
  caros_control_msgs::serial_device_move_ptp srv;
  srv.request.targets.push_back(caros::toRos(target));
  srv.request.speeds.push_back(caros::toRos(speed));
  srv.request.blends.push_back(caros::toRos(blend));

  srvMovePtp_.call<caros_control_msgs::serial_device_move_ptp>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::movePtpT(const rw::math::Transform3D<>& target, const float speed, const float blend)
{
  verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
  caros_control_msgs::serial_device_move_ptp_t srv;
  srv.request.targets.push_back(caros::toRos(target));
  srv.request.speeds.push_back(caros::toRos(speed));
  srv.request.blends.push_back(caros::toRos(blend));

  srvMovePtpT_.call<caros_control_msgs::serial_device_move_ptp_t>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveServoQ(const rw::math::Q& target, const float speed)
{
  verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
  caros_control_msgs::serial_device_move_servo_q srv;
  srv.request.targets.push_back(caros::toRos(target));
  srv.request.speeds.push_back(caros::toRos(speed));

  srvMoveServoQ_.call<caros_control_msgs::serial_device_move_servo_q>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveServoT(const rw::math::Transform3D<>& target, const float speed)
{
  verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
  caros_control_msgs::serial_device_move_servo_t srv;
  srv.request.targets.push_back(caros::toRos(target));
  srv.request.speeds.push_back(caros::toRos(speed));

  srvMoveServoT_.call<caros_control_msgs::serial_device_move_servo_t>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveVelQ(const rw::math::Q& target)
{
  caros_control_msgs::serial_device_move_vel_q srv;
  srv.request.vel = caros::toRos(target);

  srvMoveVelQ_.call<caros_control_msgs::serial_device_move_vel_q>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveVelT(const rw::math::VelocityScrew6D<>& target)
{
  caros_control_msgs::serial_device_move_vel_t srv;
  srv.request.vel = caros::toRos(target);

  srvMoveVelT_.call<caros_control_msgs::serial_device_move_vel_t>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveLinFc(const rw::math::Transform3D<>& posTarget, const rw::math::Transform3D<>& offset,
                                    const rw::math::Wrench6D<>& wrenchTarget, const rw::math::Q& controlGain)
{
  caros_control_msgs::serial_device_move_lin_fc srv;
  srv.request.pos_target = caros::toRos(posTarget);
  srv.request.offset = caros::toRos(offset);
  srv.request.wrench_target = caros::toRos(wrenchTarget);

  ROS_ASSERT(controlGain.size() == srv.request.ctrl_gains.size());
  for (std::size_t index = 0; index < srv.request.ctrl_gains.size(); ++index)
  {
    srv.request.ctrl_gains[index] = caros::toRosFloat(controlGain[index]);
  }

  srvMoveLinFc_.call<caros_control_msgs::serial_device_move_lin_fc>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::stop()
{
  caros_common_msgs::stop srv;

  srvStop_.call<caros_common_msgs::stop>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::pause()
{
  caros_common_msgs::pause srv;

  srvPause_.call<caros_common_msgs::pause>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::setSafeModeEnabled(bool enable)
{
  caros_common_msgs::config_bool srv;
  srv.request.value = caros::toRos(enable);

  srvSetSafeModeEnabled_.call<caros_common_msgs::config_bool>(srv);

  return srv.response.success;
}

/* Hardcoded since the connections are not added to a collection that can easily be iterated */
void SerialDeviceSIProxy::closePersistentConnections()
{
  srvMoveLinFc_.shutdown();
  srvMoveLin_.shutdown();
  srvMovePtp_.shutdown();
  srvMovePtpT_.shutdown();
  srvMoveServoQ_.shutdown();
  srvMoveServoT_.shutdown();
  srvMoveVelQ_.shutdown();
  srvMoveVelT_.shutdown();
  srvPause_.shutdown();
  srvStart_.shutdown();
  srvStop_.shutdown();
  srvSetSafeModeEnabled_.shutdown();
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
