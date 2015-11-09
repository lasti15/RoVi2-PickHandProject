#include <caros/serial_device_si_proxy.h>

#include <caros/serial_device_service_interface.h>
#include <caros/common.h>
#include <caros/common_robwork.h>
#include <caros_common_msgs/EmptySrv.h>
#include <caros_control_msgs/SerialDeviceMoveLin.h>
#include <caros_control_msgs/SerialDeviceMovePtp.h>
#include <caros_control_msgs/SerialDeviceMovePtpT.h>
#include <caros_control_msgs/SerialDeviceMoveVelQ.h>
#include <caros_control_msgs/SerialDeviceMoveVelT.h>
#include <caros_control_msgs/SerialDeviceMoveLinFc.h>
#include <caros_control_msgs/SerialDeviceMoveServoQ.h>
#include <caros_control_msgs/SerialDeviceMoveServoT.h>

#define SPEED_MIN 0.0f
#define SPEED_MAX 100.0f

using namespace caros;

SerialDeviceSIProxy::SerialDeviceSIProxy(ros::NodeHandle nodehandle, const std::string& devname,
                                         const bool use_persistent_connections)
    : nodehandle_(nodehandle),
      use_persistent_connections_(use_persistent_connections),
      ros_namespace_("/" + devname + "/" + SERIAL_DEVICE_SERVICE_INTERFACE_SUB_NAMESPACE),
      srv_move_lin_fc_(nodehandle_, "move_lin_fc", ros_namespace_, use_persistent_connections_),
      srv_move_lin_(nodehandle_, "move_lin", ros_namespace_, use_persistent_connections_),
      srv_move_ptp_(nodehandle_, "move_ptp", ros_namespace_, use_persistent_connections_),
      srv_move_ptp_t_(nodehandle_, "move_ptp_t", ros_namespace_, use_persistent_connections_),
      srv_move_servo_q_(nodehandle_, "move_servo_q", ros_namespace_, use_persistent_connections_),
      srv_move_servo_t_(nodehandle_, "move_servo_t", ros_namespace_, use_persistent_connections_),
      srv_move_vel_q_(nodehandle_, "move_vel_q", ros_namespace_, use_persistent_connections_),
      srv_move_vel_t_(nodehandle_, "move_vel_t", ros_namespace_, use_persistent_connections_),
      srv_stop_(nodehandle_, "move_stop", ros_namespace_, use_persistent_connections_),
      srv_set_safe_mode_enabled_(nodehandle_, "set_safe_mode_enabled", ros_namespace_, use_persistent_connections_)
{
  // states
  sub_robot_state_ =
      nodehandle_.subscribe(ros_namespace_ + "/robot_state", 1, &SerialDeviceSIProxy::handleRobotState, this);
}

SerialDeviceSIProxy::~SerialDeviceSIProxy()
{
}

bool SerialDeviceSIProxy::moveLin(const rw::math::Transform3D<>& target, const float speed, const float blend)
{
  verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
  caros_control_msgs::SerialDeviceMoveLin srv;
  srv.request.targets.push_back(caros::toRos(target));
  srv.request.speeds.push_back(caros::toRos(speed));
  srv.request.blends.push_back(caros::toRos(blend));

  srv_move_lin_.call<caros_control_msgs::SerialDeviceMoveLin>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::movePtp(const rw::math::Q& target, const float speed, const float blend)
{
  verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
  caros_control_msgs::SerialDeviceMovePtp srv;
  srv.request.targets.push_back(caros::toRos(target));
  srv.request.speeds.push_back(caros::toRos(speed));
  srv.request.blends.push_back(caros::toRos(blend));

  srv_move_ptp_.call<caros_control_msgs::SerialDeviceMovePtp>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::movePtpT(const rw::math::Transform3D<>& target, const float speed, const float blend)
{
  verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
  caros_control_msgs::SerialDeviceMovePtpT srv;
  srv.request.targets.push_back(caros::toRos(target));
  srv.request.speeds.push_back(caros::toRos(speed));
  srv.request.blends.push_back(caros::toRos(blend));

  srv_move_ptp_t_.call<caros_control_msgs::SerialDeviceMovePtpT>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveServoQ(const rw::math::Q& target, const float speed)
{
  verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
  caros_control_msgs::SerialDeviceMoveServoQ srv;
  srv.request.targets.push_back(caros::toRos(target));
  srv.request.speeds.push_back(caros::toRos(speed));

  srv_move_servo_q_.call<caros_control_msgs::SerialDeviceMoveServoQ>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveServoT(const rw::math::Transform3D<>& target, const float speed)
{
  verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
  caros_control_msgs::SerialDeviceMoveServoT srv;
  srv.request.targets.push_back(caros::toRos(target));
  srv.request.speeds.push_back(caros::toRos(speed));

  srv_move_servo_t_.call<caros_control_msgs::SerialDeviceMoveServoT>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveVelQ(const rw::math::Q& target)
{
  caros_control_msgs::SerialDeviceMoveVelQ srv;
  srv.request.vel = caros::toRos(target);

  srv_move_vel_q_.call<caros_control_msgs::SerialDeviceMoveVelQ>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveVelT(const rw::math::VelocityScrew6D<>& target)
{
  caros_control_msgs::SerialDeviceMoveVelT srv;
  srv.request.vel = caros::toRos(target);

  srv_move_vel_t_.call<caros_control_msgs::SerialDeviceMoveVelT>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::moveLinFc(const rw::math::Transform3D<>& pos_target, const rw::math::Transform3D<>& offset,
                                    const rw::math::Wrench6D<>& wrench_target, const rw::math::Q& control_gain)
{
  caros_control_msgs::SerialDeviceMoveLinFc srv;
  srv.request.pos_target = caros::toRos(pos_target);
  srv.request.offset = caros::toRos(offset);
  srv.request.wrench_target = caros::toRos(wrench_target);

  ROS_ASSERT(control_gain.size() == srv.request.ctrl_gains.size());
  for (std::size_t index = 0; index < srv.request.ctrl_gains.size(); ++index)
  {
    srv.request.ctrl_gains[index] = caros::toRosFloat(control_gain[index]);
  }

  srv_move_lin_fc_.call<caros_control_msgs::SerialDeviceMoveLinFc>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::stop()
{
  caros_common_msgs::EmptySrv srv;

  srv_stop_.call<caros_common_msgs::EmptySrv>(srv);

  return srv.response.success;
}

bool SerialDeviceSIProxy::setSafeModeEnabled(bool enable)
{
  caros_common_msgs::ConfigBool srv;
  srv.request.value = caros::toRos(enable);

  srv_set_safe_mode_enabled_.call<caros_common_msgs::ConfigBool>(srv);

  return srv.response.success;
}

/* Hardcoded since the connections are not added to a collection that can easily be iterated */
void SerialDeviceSIProxy::closePersistentConnections()
{
  srv_move_lin_fc_.shutdown();
  srv_move_lin_.shutdown();
  srv_move_ptp_.shutdown();
  srv_move_ptp_t_.shutdown();
  srv_move_servo_q_.shutdown();
  srv_move_servo_t_.shutdown();
  srv_move_vel_q_.shutdown();
  srv_move_vel_t_.shutdown();
  srv_stop_.shutdown();
  srv_set_safe_mode_enabled_.shutdown();
}

void SerialDeviceSIProxy::handleRobotState(const caros_control_msgs::RobotState& state)
{
  robot_state_ = state;
}

rw::math::Q SerialDeviceSIProxy::getQ()
{
  return caros::toRw(robot_state_.q);
}

rw::math::Q SerialDeviceSIProxy::getQd()
{
  return caros::toRw(robot_state_.dq);
}

bool SerialDeviceSIProxy::isMoving()
{
  return robot_state_.is_moving;
}

ros::Time SerialDeviceSIProxy::getTimeStamp()
{
  return robot_state_.header.stamp;
}
