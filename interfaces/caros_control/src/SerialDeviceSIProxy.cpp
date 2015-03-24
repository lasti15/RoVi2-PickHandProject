#include <caros/SerialDeviceSIProxy.hpp>

#include <caros/SerialDeviceServiceInterface.hpp>
#include <caros/common.h>
#include <caros_common_msgs/Stop.h>
#include <caros_common_msgs/Pause.h>
#include <caros_common_msgs/ConfigBool.h>
#include <caros_control_msgs/SerialDeviceMoveLin.h>
#include <caros_control_msgs/SerialDeviceMovePTP.h>
#include <caros_control_msgs/SerialDeviceMovePTP_T.h>
#include <caros_control_msgs/SerialDeviceMoveVelQ.h>
#include <caros_control_msgs/SerialDeviceMoveVelT.h>
#include <caros_control_msgs/SerialDeviceMoveLinFC.h>
#include <caros_control_msgs/SerialDeviceMoveServoQ.h>
#include <caros_control_msgs/SerialDeviceMoveServoT.h>

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
    else if (not (std::isgreaterequal(value, min) && std::islessequal(value, max)))
    {
        std::ostringstream oss;
        oss << "The value is not within [" << min << ";" << max << "]";
        throw std::range_error(oss.str());
    }
}
} // end namespace

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
    bool srvCallSuccess = false;
    verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
    caros_control_msgs::SerialDeviceMoveLin srv;
    srv.request.targets.push_back( caros::toRos(target) );
    srv.request.speeds.push_back( caros::toRos(speed) );
    srv.request.blends.push_back( caros::toRos(blend) );

    if (not _srvMoveLin.exists())
    {
        THROW_CAROS_UNAVAILABLE_SERVICE("The service " << _srvMoveLin.getService() << " does not exist.");
    }

    srvCallSuccess = _srvMoveLin.call(srv);
    if (not srvCallSuccess)
    {
        THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << _srvMoveLin.getService());
    }

    return srv.response.success;
}

bool SerialDeviceSIProxy::movePTP(const rw::math::Q& target, const float speed, const float blend)
{
    bool srvCallSuccess = false;
    verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
    caros_control_msgs::SerialDeviceMovePTP srv;
    srv.request.targets.push_back( caros::toRos(target) );
    srv.request.speeds.push_back( caros::toRos(speed) );
    srv.request.blends.push_back( caros::toRos(blend) );

    if (not _srvMovePTP.exists())
    {
        THROW_CAROS_UNAVAILABLE_SERVICE("The service " << _srvMovePTP.getService() << " does not exist.");
    }

    srvCallSuccess = _srvMovePTP.call(srv);
    if (not srvCallSuccess)
    {
        THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << _srvMovePTP.getService());
    }

    return srv.response.success;
}

bool SerialDeviceSIProxy::movePTP_T(const rw::math::Transform3D<>& target, const float speed, const float blend)
{
    bool srvCallSuccess = false;
    verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
    caros_control_msgs::SerialDeviceMovePTP_T srv;
    srv.request.targets.push_back( caros::toRos(target) );
    srv.request.speeds.push_back( caros::toRos(speed) );
    srv.request.blends.push_back( caros::toRos(blend) );

    if (not _srvMovePTP_T.exists())
    {
        THROW_CAROS_UNAVAILABLE_SERVICE("The service " << _srvMovePTP_T.getService() << " does not exist.");
    }

    srvCallSuccess = _srvMovePTP_T.call(srv);
    if (not srvCallSuccess)
    {
        THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << _srvMovePTP_T.getService());
    }

    return srv.response.success;
}

bool SerialDeviceSIProxy::moveServoQ(const rw::math::Q& target, const float speed)
{
    bool srvCallSuccess = false;
    verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
    caros_control_msgs::SerialDeviceMoveServoQ srv;
    srv.request.targets.push_back( caros::toRos(target) );
    srv.request.speeds.push_back( caros::toRos(speed) );

    if (not _srvMoveServoQ.exists())
    {
        THROW_CAROS_UNAVAILABLE_SERVICE("The service " << _srvMoveServoQ.getService() << " does not exist.");
    }

    srvCallSuccess = _srvMoveServoQ.call(srv);
    if (not srvCallSuccess)
    {
        THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << _srvMoveServoQ.getService());
    }

    return srv.response.success;
}

bool SerialDeviceSIProxy::moveServoT(const rw::math::Transform3D<>& target, const float speed)
{
    bool srvCallSuccess = false;
    verifyValueIsWithin(speed, SPEED_MIN, SPEED_MAX);
    caros_control_msgs::SerialDeviceMoveServoT srv;
    srv.request.targets.push_back( caros::toRos(target) );
    srv.request.speeds.push_back( caros::toRos(speed) );

    if (not _srvMoveServoT.exists())
    {
        THROW_CAROS_UNAVAILABLE_SERVICE("The service " << _srvMoveServoT.getService() << " does not exist.");
    }

    srvCallSuccess = _srvMoveServoT.call(srv);
    if (not srvCallSuccess)
    {
        THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << _srvMoveServoT.getService());
    }

    return srv.response.success;
}

bool SerialDeviceSIProxy::moveVelQ(const rw::math::Q& target)
{
    bool srvCallSuccess = false;
    caros_control_msgs::SerialDeviceMoveVelQ srv;
    srv.request.vel =  caros::toRos(target) ;

    if (not _srvMoveVelQ.exists())
    {
        THROW_CAROS_UNAVAILABLE_SERVICE("The service " << _srvMoveVelQ.getService() << " does not exist.");
    }

    srvCallSuccess = _srvMoveVelQ.call(srv);
    if (not srvCallSuccess)
    {
        THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << _srvMoveVelQ.getService());
    }

    return srv.response.success;
}

bool SerialDeviceSIProxy::moveVelT(const rw::math::VelocityScrew6D<>& target)
{
    bool srvCallSuccess = false;
    caros_control_msgs::SerialDeviceMoveVelT srv;
    srv.request.vel =  caros::toRos(target) ;

    if (not _srvMoveVelT.exists())
    {
        THROW_CAROS_UNAVAILABLE_SERVICE("The service " << _srvMoveVelT.getService() << " does not exist.");
    }

    srvCallSuccess = _srvMoveVelT.call(srv);
    if (not srvCallSuccess)
    {
        THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << _srvMoveVelT.getService());
    }

    return srv.response.success;
}

bool SerialDeviceSIProxy::moveLinFC(const rw::math::Transform3D<>& posTarget,
                                    const rw::math::Transform3D<>& offset,
                                    const rw::math::Wrench6D<>& wrenchTarget,
                                    const rw::math::Q& controlGain)
{
    bool srvCallSuccess = false;
    caros_control_msgs::SerialDeviceMoveLinFC srv;
    srv.request.pos_target = caros::toRos(posTarget);
    srv.request.offset = caros::toRos(offset);
    srv.request.wrench_target = caros::toRos(wrenchTarget);

    ROS_ASSERT(controlGain.size() == srv.request.ctrl_gains.size());
    for (std::size_t index = 0; index < srv.request.ctrl_gains.size(); ++index)
    {
        srv.request.ctrl_gains[index] = caros::toRosFloat(controlGain[index]);
    }

    if (not _srvMoveLinFC.exists())
    {
        THROW_CAROS_UNAVAILABLE_SERVICE("The service " << _srvMoveLinFC.getService() << " does not exist.");
    }

    srvCallSuccess = _srvMoveLinFC.call(srv);
    if (not srvCallSuccess)
    {
        THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << _srvMoveLinFC.getService());
    }

    return srv.response.success;
}

bool SerialDeviceSIProxy::stop(){
    bool srvCallSuccess = false;
    caros_common_msgs::Stop srv;

    if (not _srvStop.exists())
    {
        THROW_CAROS_UNAVAILABLE_SERVICE("The service " << _srvStop.getService() << " does not exist.");
    }

    srvCallSuccess = _srvStop.call(srv);
    if (not srvCallSuccess)
    {
        THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << _srvStop.getService());
    }

    return srv.response.success;
}

bool SerialDeviceSIProxy::pause(){
    bool srvCallSuccess = false;
    caros_common_msgs::Pause srv;

    if (not _srvPause.exists())
    {
        THROW_CAROS_UNAVAILABLE_SERVICE("The service " << _srvPause.getService() << " does not exist.");
    }

    srvCallSuccess = _srvPause.call(srv);
    if (not srvCallSuccess)
    {
        THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << _srvPause.getService());
    }

    return srv.response.success;
}

bool SerialDeviceSIProxy::setSafeModeEnabled(bool enable){
    bool srvCallSuccess = false;
    caros_common_msgs::ConfigBool srv;
    srv.request.value = caros::toRos(enable);

    if (not _srvSetSafeModeEnabled.exists())
    {
        THROW_CAROS_UNAVAILABLE_SERVICE("The service " << _srvSetSafeModeEnabled.getService() << " does not exist.");
    }

    srvCallSuccess = _srvSetSafeModeEnabled.call(srv);
    if (not srvCallSuccess)
    {
        THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << _srvSetSafeModeEnabled.getService());
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
