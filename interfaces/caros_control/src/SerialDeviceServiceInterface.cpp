/**/
#include <caros/SerialDeviceServiceInterface.hpp>
#include <caros/common.h>

#include <caros_control_msgs/RobotState.h>

#include <rw/math.hpp>
#include <boost/foreach.hpp>

#include <vector>
#include <tuple>

using namespace caros;

SerialDeviceServiceInterface::SerialDeviceServiceInterface(ros::NodeHandle nodehandle):
    _nodehandle(nodehandle, SERIAL_DEVICE_SERVICE_INTERFACE_SUB_NAMESPACE)
{
    /* Do nothing */
}

SerialDeviceServiceInterface::SerialDeviceServiceInterface() {
    /* Do nothing */
    ROS_FATAL_STREAM("The empty constructor of the GripperServiceInterface should never be called! This is undefined behaviour.");
}

SerialDeviceServiceInterface::~SerialDeviceServiceInterface() {
    /* Nothing special needs to be done - relying on ROS's RAII design */
}

bool SerialDeviceServiceInterface::configureInterface() {
    return initService();
}

bool SerialDeviceServiceInterface::initService(){
    if (_srvMoveLin || _srvMovePTP || _srvMovePTP_T || _srvMoveVelQ || _srvMoveVelT || _srvMoveServoQ || _srvMoveServoT || _srvMoveLinFC || _srvMoveStart || _srvMoveStop || _srvMovePause || _srvSetSafeModeEnabled || _deviceStatePublisher) {
        ROS_WARN_STREAM("Reinitialising one or more SerialDeviceServiceInterface services or publishers. If this is not fully intended then this should be considered a bug!");
    }

    /* TODO:
     * Should the "RobotState" not be called something like SerialDeviceState or similar?
     * ^- The name should also be replaced in the ros error stream condition statement!
     */
    _deviceStatePublisher = _nodehandle.advertise<caros_control_msgs::RobotState>("robot_state", SERIAL_DEVICE_STATE_PUBLISHER_QUEUE_SIZE);
    ROS_ERROR_STREAM_COND(!_deviceStatePublisher, "The RobotState publisher is empty!");

    _srvMoveLin = _nodehandle.advertiseService("move_lin", &SerialDeviceServiceInterface::moveLinHandle, this);
    ROS_ERROR_STREAM_COND(!_srvMoveLin, "The move_lin service is empty!");

    _srvMovePTP = _nodehandle.advertiseService("move_ptp", &SerialDeviceServiceInterface::movePTPHandle, this);
    ROS_ERROR_STREAM_COND(!_srvMovePTP, "The move_ptp service is empty!");

    _srvMovePTP_T = _nodehandle.advertiseService("move_ptp_t", &SerialDeviceServiceInterface::movePTP_THandle, this);
    ROS_ERROR_STREAM_COND(!_srvMovePTP_T, "The move_ptp_t service is empty!");

    _srvMoveVelQ = _nodehandle.advertiseService("move_vel_q", &SerialDeviceServiceInterface::moveVelQHandle, this);
    ROS_ERROR_STREAM_COND(!_srvMoveVelQ, "The move_vel_q service is empty!");

    _srvMoveVelT = _nodehandle.advertiseService("move_vel_t", &SerialDeviceServiceInterface::moveVelTHandle, this);
    ROS_ERROR_STREAM_COND(!_srvMoveVelT, "The move_vel_t service is empty!");

    _srvMoveServoQ = _nodehandle.advertiseService("move_servo_q", &SerialDeviceServiceInterface::moveServoQHandle, this);
    ROS_ERROR_STREAM_COND(!_srvMoveServoQ, "The move_servo_q service is empty!");

    _srvMoveServoT = _nodehandle.advertiseService("move_servo_t", &SerialDeviceServiceInterface::moveServoTHandle, this);
    ROS_ERROR_STREAM_COND(!_srvMoveServoT, "The move_servo_t service is empty!");

    _srvMoveLinFC = _nodehandle.advertiseService("move_lin_fc", &SerialDeviceServiceInterface::moveLinFCHandle, this);
    ROS_ERROR_STREAM_COND(!_srvMoveLinFC, "The move_lin_fc service is empty!");

    _srvMoveStart = _nodehandle.advertiseService("move_start", &SerialDeviceServiceInterface::moveStartHandle, this);
    ROS_ERROR_STREAM_COND(!_srvMoveStart, "The move_start service is empty!");

    _srvMoveStop = _nodehandle.advertiseService("move_stop", &SerialDeviceServiceInterface::moveStopHandle, this);
    ROS_ERROR_STREAM_COND(!_srvMoveStop, "The move_stop service is empty!");

    _srvMovePause = _nodehandle.advertiseService("move_pause", &SerialDeviceServiceInterface::movePauseHandle, this);
    ROS_ERROR_STREAM_COND(!_srvMovePause, "The move_pause service is empty!");

    _srvSetSafeModeEnabled = _nodehandle.advertiseService("set_safe_mode_enabled", &SerialDeviceServiceInterface::moveSetSafeModeEnabledHandle, this);
    ROS_ERROR_STREAM_COND(!_srvSetSafeModeEnabled, "The set_safe_mode_enabled service is empty!");

    if (_srvMoveLin && _srvMovePTP && _srvMovePTP_T && _srvMoveVelQ && _srvMoveVelT && _srvMoveServoQ && _srvMoveServoT && _srvMoveLinFC && _srvMoveStart && _srvMoveStop && _srvMovePause && _srvSetSafeModeEnabled && _deviceStatePublisher) {
        /* Everything seems to be properly initialised */
        ROS_DEBUG_STREAM("All SerialDeviceServiceInterface publishers and services appear to have been properly initialised");
    } else {
        ROS_ERROR_STREAM("The SerialDeviceServiceInterface could not be properly initialised - one or more ROS services or publishers failed to be properly initialised.");
        return false;
    }

    return true;
}

void SerialDeviceServiceInterface::publish(const caros_control_msgs::RobotState& state) {
    _deviceStatePublisher.publish(state);
}

/* FIXME:
 * Remove hardcoded information regarding the targets and speeds are vectors or (random) indexing data structures  e.g. targets_t::size_t looping and such...
 */
template <typename targets_t, typename speeds_t, typename container_t>
bool fillContainerWithTargetsAndSpeeds(const targets_t& targets, const speeds_t& speeds, container_t& res) {
    if (targets.size() != speeds.size()) {
        ROS_WARN_STREAM("There are " << targets.size() << " targets and " << speeds.size() << " speeds, but there should be the same amount of each!");
        return false;
    }

    ROS_ASSERT(targets.size() == speeds.size()); /* Just an extra "safety" control to verify that our double indexing loop will behave as expected */
    res.clear();
    res.reserve(targets.size());

    /* TODO:
     * Perform the for-loop within a try-catch block to catch out-of-range access within the .at() call
     */
    for (typename targets_t::size_type index = 0; index < targets.size(); ++index) {
        res.push_back(std::make_tuple(caros::toRw(targets.at(index)), speeds.at(index)));
    }

    return true;
}

/************************************************************************
 * ROS service handle functions
 ************************************************************************/
/* TODO:
 * Rewrite the functions to also take in the blends (which should be added to the .srv files). Also remember to update the non-handle versions.
 * Try to refactor as much of the shared conversion/code for all these methods, into separate functions, like with the fillContainerWithTransformsAndSpeed(...) function.
 */
bool SerialDeviceServiceInterface::moveLinHandle(caros_control_msgs::SerialDeviceMoveLin::Request& request,
                                                 caros_control_msgs::SerialDeviceMoveLin::Response& response)
{
    TransformAndSpeedContainer_t res;
    if (fillContainerWithTargetsAndSpeeds(request.targets, request.speeds, res)) {
        response.success = moveLin(res);
    } else {
        response.success = false;
    }

    return true;
}

bool SerialDeviceServiceInterface::movePTPHandle(caros_control_msgs::SerialDeviceMovePTP::Request& request,
                                                 caros_control_msgs::SerialDeviceMovePTP::Response& response)
{
    QAndSpeedContainer_t res;
    if (fillContainerWithTargetsAndSpeeds(request.targets, request.speeds, res)) {
        response.success = movePTP(res);
    } else {
        response.success = false;
    }

    return true;
}

bool SerialDeviceServiceInterface::movePTP_THandle(caros_control_msgs::SerialDeviceMovePTP_T::Request& request,
                                                   caros_control_msgs::SerialDeviceMovePTP_T::Response& response)
{
    TransformAndSpeedContainer_t res;
    if (fillContainerWithTargetsAndSpeeds(request.targets, request.speeds, res)) {
        response.success = movePTP_T(res);
    } else {
        response.success = false;
    }

    return true;
}    

bool SerialDeviceServiceInterface::moveVelQHandle(caros_control_msgs::SerialDeviceMoveVelQ::Request& request,
                                                  caros_control_msgs::SerialDeviceMoveVelQ::Response& response)
{
    rw::math::Q vel = caros::toRw(request.vel);
    response.success = moveVelQ(vel);

    return true;
}

bool SerialDeviceServiceInterface::moveVelTHandle(caros_control_msgs::SerialDeviceMoveVelT::Request& request,
                                                  caros_control_msgs::SerialDeviceMoveVelT::Response& response)
{
    rw::math::VelocityScrew6D<> vel = caros::toRw(request.vel);
    response.success = moveVelT(vel);

    return true;
}

/* TODO:
 * Add speeds implementation
 * This should also be added to the RobWorkHardware URCallBackInterface servo functioninality, where the speed should be optional, defaulting to the currently hardcoded value...
 */
bool SerialDeviceServiceInterface::moveServoQHandle(caros_control_msgs::SerialDeviceMoveServoQ::Request& request,
                                                    caros_control_msgs::SerialDeviceMoveServoQ::Response& response)
{
    QAndSpeedContainer_t res;
    if (fillContainerWithTargetsAndSpeeds(request.targets, request.speeds, res)) {
        response.success = moveServoQ(res);
    } else {
        response.success = false;
    }

    return true;
}

bool SerialDeviceServiceInterface::moveServoTHandle(caros_control_msgs::SerialDeviceMoveServoT::Request& request,
                                                    caros_control_msgs::SerialDeviceMoveServoT::Response& response)
{
    TransformAndSpeedContainer_t res;
    if (fillContainerWithTargetsAndSpeeds(request.targets, request.speeds, res)) {
        response.success = moveServoT(res);
    } else {
        response.success = false;
    }

    return true;
}

bool SerialDeviceServiceInterface::moveLinFCHandle(caros_control_msgs::SerialDeviceMoveLinFC::Request& request,
                                                   caros_control_msgs::SerialDeviceMoveLinFC::Response& response)
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
//    ROS_ASSERT_MSG(request.ctrl_gains.size() == 6, "The size of ctrl_gains " << request.ctrl_gains.size() << " but should be 6!");
    ROS_ASSERT(request.ctrl_gains.size() == 6);
    /* FIXME:
     * Convert the ctrl_gains (boost::array) to std::vector or rw::math::Q ?
     */
    rw::math::Q selection(request.ctrl_gains.size(), request.ctrl_gains.at(0), request.ctrl_gains.at(1), request.ctrl_gains.at(2), request.ctrl_gains.at(3), request.ctrl_gains.at(4), request.ctrl_gains.at(5));

    response.success = moveLinFC(posTarget, offset, wrenchTarget, selection);

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
