#ifndef URSERVICEINTERFACE_HPP
#define URSERVICEINTERFACE_HPP

#include <caros_universalrobot/URServiceServoQ.h>
#include <caros_universalrobot/URServiceServoT.h>
#include <caros_universalrobot/URServiceEmpty.h>
#include <caros_universalrobot/URServiceForceModeUpdate.h>
#include <caros_universalrobot/URServiceForceModeStart.h>
#include <caros_universalrobot/URServiceForceModeStop.h>


#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Wrench6D.hpp>

#include <ros/ros.h>

#define URSERVICEINTERFACE_SUB_NAMESPACE "ur_service_interface"

/* TODO:
 * Should this be placed inside a namespace, and should it be caros, caros::ur or just ur?
 */

class URServiceInterface {
public:
    /**
     * @brief constructor
     * @param[in] nodehandle the nodehandle to use for services.
     */
    URServiceInterface(const ros::NodeHandle& nodehandle);

    /**
     * @brief virtual destructor
     */
    virtual ~URServiceInterface();

    /**
     * @brief setup the ROS services for this interface
     */
    bool configureURService();

    /**
     * @brief teardown the ROS services for this interface
     */
    bool cleanupURService();

    /**
     * @brief move robot using a pose as target (requires inverse kinematics)
     */
    virtual bool servoT(const rw::math::Transform3D<>& target) = 0;

    /**
     * @brief move robot using an Q-configuration as target
     */
    virtual bool servoQ(const rw::math::Q& target) = 0;

    /* TODO:
     * 'force mode stop' doesn't seem to do anything within the RobWorkHardware/src/rwhw/universalrobot/URCallBackInterface.cpp file...
     */
    /* FIXME:
     * Properly document what these functions do and when/why they should be used
     */
    virtual bool forceModeStart(const rw::math::Transform3D<>& refToffset, const rw::math::Q& selection, const rw::math::Wrench6D<>& wrenchTarget, const rw::math::Q& limits) = 0;
    virtual bool forceModeUpdate(const rw::math::Wrench6D<>& wrenchTarget) = 0;
    virtual bool forceModeStop() = 0;

private:
    bool servoTHandle(caros_universalrobot::URServiceServoT::Request& request, caros_universalrobot::URServiceServoT::Response& response);

    bool servoQHandle(caros_universalrobot::URServiceServoQ::Request& request, caros_universalrobot::URServiceServoQ::Response& response);

    bool forceModeStartHandle(caros_universalrobot::URServiceForceModeStart::Request& request, caros_universalrobot::URServiceForceModeStart::Response& response);

    bool forceModeUpdateHandle(caros_universalrobot::URServiceForceModeUpdate::Request& request, caros_universalrobot::URServiceForceModeUpdate::Response& response);

    bool forceModeStopHandle(caros_universalrobot::URServiceForceModeStop::Request& request, caros_universalrobot::URServiceForceModeStop::Response& response);

protected:
    ros::NodeHandle _nodehandle;

    ros::ServiceServer _srvServoT;
    ros::ServiceServer _srvServoQ;
    ros::ServiceServer _srvForceModeStart;
    ros::ServiceServer _srvForceModeUpdate;
    ros::ServiceServer _srvForceModeStop;

};

#endif //#ifndef URSERVICEINTERFACE_HPP
