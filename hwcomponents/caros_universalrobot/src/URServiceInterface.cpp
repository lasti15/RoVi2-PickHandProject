#include <caros/URServiceInterface.hpp>

#include <caros/common.h>

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>

#include <ros/ros.h>

#include <algorithm>

URServiceInterface::URServiceInterface(const ros::NodeHandle& nodehandle):
    _nodehandle(nodehandle)
{
    /* Do nothing for now */
    /* No way to verify that this object is properly configured or just a zombie object, since RAII is not being used */
}

URServiceInterface::~URServiceInterface() {
    /* Currently no special things to clean up */
}

bool URServiceInterface::configureURService() {
    if (_srvServoT || _srvServoQ || _srvForceModeStart || _srvForceModeUpdate || _srvForceModeStop) {
        ROS_WARN_STREAM("Reinitialising one or more URServiceInterface services. If this is not fully intended then this should be considered a bug!");
    }

    _srvServoT = _nodehandle.advertiseService("servo_t", &URServiceInterface::servoTHandle, this);
    ROS_ERROR_STREAM_COND(!_srvServoT, "The servo_t service is empty!");

    _srvServoQ = _nodehandle.advertiseService("servo_q", &URServiceInterface::servoQHandle, this);
    ROS_ERROR_STREAM_COND(!_srvServoQ, "The servo_q service is empty!");

    _srvForceModeStart = _nodehandle.advertiseService("force_mode_start", &URServiceInterface::forceModeStartHandle, this);
    ROS_ERROR_STREAM_COND(!_srvForceModeStart, "The force_mode_start service is empty!");

    _srvForceModeUpdate = _nodehandle.advertiseService("force_mode_update", &URServiceInterface::forceModeUpdateHandle, this);
    ROS_ERROR_STREAM_COND(!_srvForceModeUpdate, "The force_mode_update service is empty!");

    _srvForceModeStop = _nodehandle.advertiseService("force_mode_stop", &URServiceInterface::forceModeStopHandle, this);
    ROS_ERROR_STREAM_COND(!_srvForceModeStop, "The force_mode_stop service is empty!");

    if (_srvServoT && _srvServoQ && _srvForceModeStart && _srvForceModeUpdate && _srvForceModeStop) {
        /* Everything seems to have been properly initialised */
    } else {
        ROS_ERROR_STREAM("The URService could not be properly initialised - one or more ros services may not be up and running or working as intended!");
        return false;
    }

    return true;
}

bool URServiceInterface::cleanupURService() {
    if (_srvServoT) {
        _srvServoT.shutdown();
    } else {
        ROS_ERROR_STREAM("While trying to cleanup the URService, _srvServoT was empty!");
    }
    if (_srvServoQ) {
        _srvServoQ.shutdown();
    } else {
        ROS_ERROR_STREAM("While trying to cleanup the URService, _srvServoQ was empty!");
    }
    if (_srvForceModeStart) {
        _srvForceModeStart.shutdown();
    } else {
        ROS_ERROR_STREAM("While trying to cleanup the URService, _srvForceModeStart was empty!");
    }
    if (_srvForceModeUpdate) {
        _srvForceModeUpdate.shutdown();
    } else {
        ROS_ERROR_STREAM("While trying to cleanup the URService, _srvForceModeUpdate was empty!");
    }
    if (_srvForceModeStop) {
        _srvForceModeStop.shutdown();
    } else {
        ROS_ERROR_STREAM("While trying to cleanup the URService, _srvForceModeStop was empty!");
    }

    return true;
}

bool URServiceInterface::servoTHandle(caros_universalrobot::URServiceServoT::Request& request, caros_universalrobot::URServiceServoT::Response& response) {
    rw::math::Transform3D<> target = caros::toRw(request.target);
    response.success = servoT(target);
    return true;
}

bool URServiceInterface::servoQHandle(caros_universalrobot::URServiceServoQ::Request& request, caros_universalrobot::URServiceServoQ::Response& response) {
    rw::math::Q target = caros::toRw(request.target);
    response.success = servoQ(target);
    return true;
}

bool URServiceInterface::forceModeStartHandle(caros_universalrobot::URServiceForceModeStart::Request& request, caros_universalrobot::URServiceForceModeStart::Response& response) {
    rw::math::Transform3D<> refToffset = caros::toRw(request.base2forceFrame);
    rw::math::Wrench6D<> wrenchTarget;
    wrenchTarget(0) = request.wrench.force.x;
    wrenchTarget(1) = request.wrench.force.y;
    wrenchTarget(2) = request.wrench.force.z;

    wrenchTarget(3) = request.wrench.torque.x;
    wrenchTarget(4) = request.wrench.torque.y;
    wrenchTarget(5) = request.wrench.torque.z;

    std::size_t index;
    rw::math::Q selection(request.selection.size());
    index = 0;
    for (const auto item : request.selection) {
        selection(index++) = static_cast<double>(item);
    }

    rw::math::Q limits(request.limits.size());
    index = 0;
    for (const auto item : request.limits) {
        limits(index++) = static_cast<double>(item);
    }

    response.success = forceModeStart(refToffset, selection, wrenchTarget, limits);
    return true;
}

bool URServiceInterface::forceModeUpdateHandle(caros_universalrobot::URServiceForceModeUpdate::Request& request, caros_universalrobot::URServiceForceModeUpdate::Response& response) {
    rw::math::Wrench6D<> wrenchTarget;
    wrenchTarget(0) = request.wrench.force.x;
    wrenchTarget(1) = request.wrench.force.y;
    wrenchTarget(2) = request.wrench.force.z;

    wrenchTarget(3) = request.wrench.torque.x;
    wrenchTarget(4) = request.wrench.torque.y;
    wrenchTarget(5) = request.wrench.torque.z;

    response.success = forceModeUpdate(wrenchTarget);
    return true;
}

bool URServiceInterface::forceModeStopHandle(caros_universalrobot::URServiceForceModeStop::Request& request, caros_universalrobot::URServiceForceModeStop::Response& response) {
    response.success = forceModeStop();
    return true;
}
