#include <caros/ur_service_interface.h>

#include <caros/common.h>

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>

#include <ros/ros.h>

#include <algorithm>

URServiceInterface::URServiceInterface(const ros::NodeHandle& nodehandle):
    nodehandle_(nodehandle)
{
    /* Do nothing for now */
    /* No way to verify that this object is properly configured or just a zombie object, since RAII is not being used */
}

URServiceInterface::~URServiceInterface() {
    /* Currently no special things to clean up */
}

bool URServiceInterface::configureURService() {
    if (srvServoT_ || srvServoQ_ || srvForceModeStart_ || srvForceModeUpdate_ || srvForceModeStop_) {
        ROS_WARN_STREAM("Reinitialising one or more URServiceInterface services. If this is not fully intended then this should be considered a bug!");
    }

    srvServoT_ = nodehandle_.advertiseService("servo_t", &URServiceInterface::servoTHandle, this);
    ROS_ERROR_STREAM_COND(!srvServoT_, "The servo_t service is empty!");

    srvServoQ_ = nodehandle_.advertiseService("servo_q", &URServiceInterface::servoQHandle, this);
    ROS_ERROR_STREAM_COND(!srvServoQ_, "The servo_q service is empty!");

    srvForceModeStart_ = nodehandle_.advertiseService("force_mode_start", &URServiceInterface::forceModeStartHandle, this);
    ROS_ERROR_STREAM_COND(!srvForceModeStart_, "The force_mode_start service is empty!");

    srvForceModeUpdate_ = nodehandle_.advertiseService("force_mode_update", &URServiceInterface::forceModeUpdateHandle, this);
    ROS_ERROR_STREAM_COND(!srvForceModeUpdate_, "The force_mode_update service is empty!");

    srvForceModeStop_ = nodehandle_.advertiseService("force_mode_stop", &URServiceInterface::forceModeStopHandle, this);
    ROS_ERROR_STREAM_COND(!srvForceModeStop_, "The force_mode_stop service is empty!");

    if (srvServoT_ && srvServoQ_ && srvForceModeStart_ && srvForceModeUpdate_ && srvForceModeStop_) {
        /* Everything seems to have been properly initialised */
    } else {
        ROS_ERROR_STREAM("The URService could not be properly initialised - one or more ros services may not be up and running or working as intended!");
        return false;
    }

    return true;
}

bool URServiceInterface::cleanupURService() {
    if (srvServoT_) {
        srvServoT_.shutdown();
    } else {
        ROS_ERROR_STREAM("While trying to cleanup the URService, srvServoT_ was empty!");
    }
    if (srvServoQ_) {
        srvServoQ_.shutdown();
    } else {
        ROS_ERROR_STREAM("While trying to cleanup the URService, srvServoQ_ was empty!");
    }
    if (srvForceModeStart_) {
        srvForceModeStart_.shutdown();
    } else {
        ROS_ERROR_STREAM("While trying to cleanup the URService, srvForceModeStart_ was empty!");
    }
    if (srvForceModeUpdate_) {
        srvForceModeUpdate_.shutdown();
    } else {
        ROS_ERROR_STREAM("While trying to cleanup the URService, srvForceModeUpdate_ was empty!");
    }
    if (srvForceModeStop_) {
        srvForceModeStop_.shutdown();
    } else {
        ROS_ERROR_STREAM("While trying to cleanup the URService, srvForceModeStop_ was empty!");
    }

    return true;
}

bool URServiceInterface::servoTHandle(caros_universalrobot::ur_service_servo_t::Request& request, caros_universalrobot::ur_service_servo_t::Response& response) {
    rw::math::Transform3D<> target = caros::toRw(request.target);
    response.success = servoT(target);
    return true;
}

bool URServiceInterface::servoQHandle(caros_universalrobot::ur_service_servo_q::Request& request, caros_universalrobot::ur_service_servo_q::Response& response) {
    rw::math::Q target = caros::toRw(request.target);
    response.success = servoQ(target);
    return true;
}

bool URServiceInterface::forceModeStartHandle(caros_universalrobot::ur_service_force_mode_start::Request& request, caros_universalrobot::ur_service_force_mode_start::Response& response) {
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

bool URServiceInterface::forceModeUpdateHandle(caros_universalrobot::ur_service_force_mode_update::Request& request, caros_universalrobot::ur_service_force_mode_update::Response& response) {
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

bool URServiceInterface::forceModeStopHandle(caros_universalrobot::ur_service_force_mode_stop::Request& request, caros_universalrobot::ur_service_force_mode_stop::Response& response) {
    response.success = forceModeStop();
    return true;
}
