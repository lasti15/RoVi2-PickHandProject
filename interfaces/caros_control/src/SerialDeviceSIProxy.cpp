#include <caros/SerialDeviceSIProxy.hpp>

//#include <fstream>

#include <caros/SerialDeviceServiceInterface.hpp>

using namespace carso;

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
    _srvMoveStart = _nodehandle.serviceClient<caros_common_msgs::EmptySrv> (rosNamespace.str() + "/move_start");
    _srvMoveStop = _nodehandle.serviceClient<caros_common_msgs::EmptySrv> (rosNamespace.str() + "/move_stop");
    _srvMovePause = _nodehandle.serviceClient<caros_common_msgs::EmptySrv> (rosNamespace.str() + "/move_pause");
    _srvMoveSafe = _nodehandle.serviceClient<caros_common_msgs::ConfigBool> (rosNamespace.str() + "/set_safe_mode_enabled");

    // states
    _subRobotState = _nodehandle.subscribe(rosNamespace.str() + "/robot_state", 1, &SerialDeviceSIProxy::handleRobotState, this);
}

SerialDeviceSIProxy::~SerialDeviceSIProxy() {
}

bool SerialDeviceSIProxy::moveLin(const rw::math::Transform3D<>& target, float speed, float blend)
{
    caros_control_msgs::SerialDeviceMoveLin srv;
    srv.request.targets.push_back( caros::toRos(target) );
    srv.request.speeds.push_back( caros::toRos(speed) );
    srv.request.blends.push_back( caros::toRos(blend) );
    return _srvMovePTP_T.call(srv);
}

bool SerialDeviceSIProxy::movePTP(const rw::math::Q& target, float speed, float blend)
{
    caros_control_msgs::SerialDeviceMovePTP srv;
    srv.request.targets.push_back( caros::toRos(target) );
    srv.request.speeds.push_back( caros::toRos(speed) );
    srv.request.blends.push_back( caros::toRos(blend) );
    return _srvMovePTP.call(srv);
}

bool SerialDeviceSIProxy::movePTP_T(const rw::math::Transform3D<>& target, float speed, float blend)
{
    caros_control_msgs::SerialDeviceMovePTP_T srv;
    srv.request.targets.push_back( caros::toRos(target) );
    srv.request.speeds.push_back( caros::toRos(speed) );
    srv.request.blends.push_back( caros::toRos(blend) );
    return _srvMovePTP_T.call(srv);
}

bool SerialDeviceSIProxy::servoQ(const rw::math::Q& target, float speed)
{
    caros_control_msgs::SerialDeviceMovePTP srv;
    srv.request.targets.push_back( caros::toRos(target) );
    srv.request.speeds.push_back( caros::toRos(speed) );
    return _srvServoQ.call(srv);
}

bool SerialDeviceSIProxy::servoT(const rw::math::Transform3D<>& target, float speed)
{
    caros_control_msgs::SerialDeviceMovePTP_T srv;
    srv.request.targets.push_back( caros::toRos(target) );
    srv.request.speeds.push_back( caros::toRos(speed) );
    return _srvServoT.call(srv);
}

bool SerialDeviceSIProxy::moveVelQ(const rw::math::Q& target)
{
    caros_control_msgs::SerialDeviceMoveVelQ srv;
    srv.request.q_vel =  caros::toRos(target) ;
    return _srvMoveVelQ.call(srv);
}

bool SerialDeviceSIProxy::moveVelT(const rw::math::VelocityScrew6D<>& target)
{
    caros_control_msgs::SerialDeviceMoveVelT srv;
    srv.request.vel =  caros::toRos(target) ;
    return _srvMoveVelT.call(srv);
}

bool SerialDeviceSIProxy::moveLinFC(const rw::math::Transform3D<>& target,
                                    rw::math::Wrench6D<>& wtarget,
                                    float selection[6],
                                    std::string refframe,
                                    rw::math::Transform3D<> offset,
                                    float speed,
                                    float blend)
{
    //! TODO: need implementing
    ROS_ERROR("NOT IMPLEMENTED!");
    return false;
}

bool SerialDeviceSIProxy::stop(){
    caros_control_msgs::Stop srv;
    return _srvStop.call(srv);
}

bool SerialDeviceSIProxy::pause(){
    caros_control_msgs::Pause srv;
    return _srvPause.call(srv);
}

bool SerialDeviceSIProxy::setSafeModeEnabled(bool enable){
    caros_control_msgs::ConfigBool srv;
    srv.request.value = caros::toRos(enable);
    return _srvStop.call(srv);
}

void SerialDeviceSIProxy::handleRobotState(const caros_control_msgs::RobotState& state) {
    boost::mutex::scoped_lock lock(_mutex);
    _q = caros::toRw(state.q);
    _dq = caros::toRw(state.dq);
    _pRobotState = state;
}

rw::math::Q SerialDeviceSIProxy::getQ() {
    boost::mutex::scoped_lock lock(_mutex);
    return _q;
}

rw::math::Q SerialDeviceSIProxy::getQd() {
    boost::mutex::scoped_lock lock(_mutex);
    return _dq;
}

bool SerialDeviceSIProxy::isMoving() {
    boost::mutex::scoped_lock lock(_mutex);
    return _pRobotState.isMoving;
}

ros::Time SerialDeviceSIProxy::getTimeStamp() {
    boost::mutex::scoped_lock lock(_mutex);
    return _pRobotState.header.stamp;
}
