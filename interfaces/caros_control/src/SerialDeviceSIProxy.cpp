#include <caros/SerialDeviceSIProxy.hpp>

#include <caros/SerialDeviceServiceInterface.hpp>
#include <caros/common.h>
#include <caros_common_msgs/Stop.h>
#include <caros_common_msgs/Pause.h>
#include <caros_common_msgs/ConfigBool.h>


using namespace caros;

SerialDeviceSIProxy::SerialDeviceSIProxy(ros::NodeHandle nodehandle, const std::string& devname):
    _nodehandle(nodehandle)
{
    std::ostringstream rosNamespace;
    rosNamespace << "/" << devname << "/" << SERIAL_DEVICE_SERVICE_INTERFACE_SUB_NAMESPACE;
    
    /* TODO:
     * Consider persistent connections - especially for the servoQ - or make it a configurable parameter/argument
     */
    _srvMoveLin = _nodehandle.serviceClient<caros_control_msgs::SerialDeviceMoveLin> (rosNamespace.str() + "/move_lin");
    _srvMovePTP = _nodehandle.serviceClient<caros_control_msgs::SerialDeviceMovePTP> (rosNamespace.str() + "/move_ptp");
    _srvMovePTP_T = _nodehandle.serviceClient<caros_control_msgs::SerialDeviceMovePTP_T> (rosNamespace.str() + "/move_ptp_t");
    _srvMoveVelQ = _nodehandle.serviceClient<caros_control_msgs::SerialDeviceMoveVelQ> (rosNamespace.str() + "/move_vel_q");
    _srvMoveVelT = _nodehandle.serviceClient<caros_control_msgs::SerialDeviceMoveVelT> (rosNamespace.str() + "/move_vel_t");
    _srvMoveServoQ = _nodehandle.serviceClient<caros_control_msgs::SerialDeviceMoveServoQ> (rosNamespace.str() + "/move_servo_q");
    _srvMoveServoT = _nodehandle.serviceClient<caros_control_msgs::SerialDeviceMoveServoT> (rosNamespace.str() + "/move_servo_t");
    _srvMoveLinFC = _nodehandle.serviceClient<caros_control_msgs::SerialDeviceMoveLinFC> (rosNamespace.str() + "/move_lin_fc");
    _srvStart = _nodehandle.serviceClient<caros_common_msgs::EmptySrv> (rosNamespace.str() + "/move_start");
    _srvStop = _nodehandle.serviceClient<caros_common_msgs::EmptySrv> (rosNamespace.str() + "/move_stop");
    _srvPause = _nodehandle.serviceClient<caros_common_msgs::EmptySrv> (rosNamespace.str() + "/move_pause");
    _srvSetSafeModeEnabled = _nodehandle.serviceClient<caros_common_msgs::ConfigBool> (rosNamespace.str() + "/set_safe_mode_enabled");

    // states
    _subRobotState = _nodehandle.subscribe(rosNamespace.str() + "/robot_state", 1, &SerialDeviceSIProxy::handleRobotState, this);
}

SerialDeviceSIProxy::~SerialDeviceSIProxy() {
}

bool SerialDeviceSIProxy::moveLin(const rw::math::Transform3D<>& target, const float speed, const float blend)
{
    caros_control_msgs::SerialDeviceMoveLin srv;
    srv.request.targets.push_back( caros::toRos(target) );
    srv.request.speeds.push_back( caros::toRos(speed) );
    srv.request.blends.push_back( caros::toRos(blend) );
    if (not _srvMoveLin.call(srv))
    {
      ROS_ERROR("The service call failed!");
    }
    return srv.response.success;
}

bool SerialDeviceSIProxy::movePTP(const rw::math::Q& target, const float speed, const float blend)
{
    caros_control_msgs::SerialDeviceMovePTP srv;
    srv.request.targets.push_back( caros::toRos(target) );
    srv.request.speeds.push_back( caros::toRos(speed) );
    srv.request.blends.push_back( caros::toRos(blend) );
    if (not _srvMovePTP.call(srv))
    {
      ROS_ERROR("The service call failed!");
    }
    return srv.response.success;
}

bool SerialDeviceSIProxy::movePTP_T(const rw::math::Transform3D<>& target, const float speed, const float blend)
{
    caros_control_msgs::SerialDeviceMovePTP_T srv;
    srv.request.targets.push_back( caros::toRos(target) );
    srv.request.speeds.push_back( caros::toRos(speed) );
    srv.request.blends.push_back( caros::toRos(blend) );
    if (not _srvMovePTP_T.call(srv))
    {
      ROS_ERROR("The service call failed!");
    }
    return srv.response.success;
}

bool SerialDeviceSIProxy::moveServoQ(const rw::math::Q& target, const float speed)
{
    caros_control_msgs::SerialDeviceMoveServoQ srv;
    srv.request.targets.push_back( caros::toRos(target) );
    srv.request.speeds.push_back( caros::toRos(speed) );
    if (not _srvMoveServoQ.call(srv))
    {
      ROS_ERROR("The service call failed!");
    }
    return srv.response.success;
}

bool SerialDeviceSIProxy::moveServoT(const rw::math::Transform3D<>& target, const float speed)
{
    caros_control_msgs::SerialDeviceMoveServoT srv;
    srv.request.targets.push_back( caros::toRos(target) );
    srv.request.speeds.push_back( caros::toRos(speed) );
    if (not _srvMoveServoT.call(srv))
    {
      ROS_ERROR("The service call failed!");
    }
    return srv.response.success;
}

bool SerialDeviceSIProxy::moveVelQ(const rw::math::Q& target)
{
    caros_control_msgs::SerialDeviceMoveVelQ srv;
    srv.request.vel =  caros::toRos(target) ;
    if (not _srvMoveVelQ.call(srv))
    {
      ROS_ERROR("The service call failed!");
    }
    return srv.response.success;
}

bool SerialDeviceSIProxy::moveVelT(const rw::math::VelocityScrew6D<>& target)
{
    caros_control_msgs::SerialDeviceMoveVelT srv;
    srv.request.vel =  caros::toRos(target) ;
    if (not _srvMoveVelT.call(srv))
    {
      ROS_ERROR("The service call failed!");
    }
    return srv.response.success;
}

bool SerialDeviceSIProxy::moveLinFC(const rw::math::Transform3D<>& target,
                                    const rw::math::Wrench6D<>& wtarget,
                                    const float selection[6],
                                    const std::string refframe,
                                    const rw::math::Transform3D<> offset,
                                    const float speed,
                                    const float blend)
{
    //! TODO: need implementing
    ROS_ERROR("NOT IMPLEMENTED!");
    return false;
}

bool SerialDeviceSIProxy::stop(){
    caros_common_msgs::Stop srv;
    if (not _srvStop.call(srv))
    {
      ROS_ERROR("The service call failed!");
    }
    return srv.response.success;
}

bool SerialDeviceSIProxy::pause(){
    caros_common_msgs::Pause srv;
    if (not _srvPause.call(srv))
    {
      ROS_ERROR("The service call failed!");
    }
    return srv.response.success;
}

bool SerialDeviceSIProxy::setSafeModeEnabled(bool enable){
    caros_common_msgs::ConfigBool srv;
    srv.request.value = caros::toRos(enable);
    if (not _srvSetSafeModeEnabled.call(srv))
    {
      ROS_ERROR("The service call failed!");
    }
    return srv.response.success;
}

void SerialDeviceSIProxy::handleRobotState(const caros_control_msgs::RobotState& state) {
    _pRobotState = state;
}

rw::math::Q SerialDeviceSIProxy::getQ() {
    return caros::toRw(_pRobotState.q);
}

rw::math::Q SerialDeviceSIProxy::getQd() {
    return caros::toRw(_pRobotState.dq);
}

bool SerialDeviceSIProxy::isMoving() {
    return _pRobotState.isMoving;
}

ros::Time SerialDeviceSIProxy::getTimeStamp() {
    return _pRobotState.header.stamp;
}
