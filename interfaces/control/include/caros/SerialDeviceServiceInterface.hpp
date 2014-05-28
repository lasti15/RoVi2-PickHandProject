/**/
#ifndef SERIALDEVICESERVICEINTERFACE_HPP
#define SERIALDEVICESERVICEINTERFACE_HPP

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/VelocityScrew6D.hpp>
#include <rw/common/Ptr.hpp>

#include <caros_control/RobotState.h>
#include <caros_control/SerialDeviceMoveLin.h>
#include <caros_control/SerialDeviceMovePTP.h>
#include <caros_control/SerialDeviceMovePTP_T.h>
#include <caros_control/SerialDeviceMoveVelQ.h>
#include <caros_control/SerialDeviceMoveVelT.h>
#include <caros_control/SerialDeviceMoveLinFC.h>
#include <caros_common/ConfigBool.h>
#include <std_srvs/Empty.h>

#include <ros/ros.h>
#include <string>

/**
 * @brief this is the top level robot arm interface. It defines the
 * minimum interface that a joint based robotic arm device needs
 * to implement.
 *
 * The namespace of the nodehandle/service_name is used and it is important that
 * not two RobotArmServiceInterfaces are running in the same namespace.
 */
class SerialDeviceServiceInterface {
public:
    typedef rw::common::Ptr<SerialDeviceServiceInterface> Ptr;
	SerialDeviceServiceInterface(const std::string& service_name);

	SerialDeviceServiceInterface(ros::NodeHandle nodeHnd);

	virtual ~SerialDeviceServiceInterface();
protected:

	//! initialize services in the node handle
	void initNodeHandle();


	//! @brief move robot in a linear Cartesian path
	virtual bool moveLin(caros_control::SerialDeviceMoveLin::Request& request,
	                     caros_control::SerialDeviceMoveLin::Response& response) = 0;

	//! @brief move robot from point to point
	virtual bool movePTP(caros_control::SerialDeviceMovePTP::Request& request,
	                     caros_control::SerialDeviceMovePTP::Response& response) = 0;

	//! @brief move robot from point to point but using a pose as target (require invkin)
	virtual bool movePTP_T(caros_control::SerialDeviceMovePTP_T::Request& request,
	                       caros_control::SerialDeviceMovePTP_T::Response& response) = 0;

	//! @brief move robot in a servoing fasion specifying joint velocity targets
	virtual bool moveVelQ(caros_control::SerialDeviceMoveVelQ::Request& request,
	                      caros_control::SerialDeviceMoveVelQ::Response& response) = 0;

	//! @brief move robot in a servoing fasion specifying a velocity screw in tool coordinates
	virtual bool moveVelT(caros_control::SerialDeviceMoveVelT::Request& request,
	                      caros_control::SerialDeviceMoveVelT::Response& response) = 0;

	//! @brief move robot in a servoing fasion specifying joint velocity targets
	virtual bool servoQ(caros_control::SerialDeviceMovePTP::Request& request,
	                    caros_control::SerialDeviceMovePTP::Response& response) = 0;

	//! @brief move robot in a servoing fasion specifying a velocity screw in tool coordinates
	virtual bool servoT(caros_control::SerialDeviceMovePTP_T::Request& request,
	                    caros_control::SerialDeviceMovePTP_T::Response& response) = 0;


	//! move robot with a hybrid position/force control
	virtual bool moveLinFC(caros_control::SerialDeviceMoveLinFC::Request& request,
	                       caros_control::SerialDeviceMoveLinFC::Response& response) = 0;

	//! hard stop the robot,
	virtual bool start(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) = 0;

	//! hard stop the robot,
	virtual bool stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) = 0;

	//! pause the robot, should be able to continue trajectory
	virtual bool pause(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) = 0;

	//! enable safe mode, so that robot stops when collisions are detected
	virtual bool setSafeModeEnabled(caros_common::ConfigBool::Request& request,
	                                    caros_common::ConfigBool::Response& response) = 0;

	//! publish robot state
	void publish(const caros_control::RobotState& state);

protected:
	std::string _service_name;
	ros::NodeHandle _nodeHnd;


    ros::Publisher _deviceStatePublisher;

    ros::ServiceServer _srvMovePTP;
    ros::ServiceServer _srvMovePTP_T;
    ros::ServiceServer _srvMoveLin;
    ros::ServiceServer _srvMoveLinFC;
    ros::ServiceServer _srvMoveVelQ;
    ros::ServiceServer _srvMoveVelT;
    ros::ServiceServer _srvServoQ;
    ros::ServiceServer _srvServoT;

    ros::ServiceServer _srvStop;
    ros::ServiceServer _srvPause;
    ros::ServiceServer _srvSafe;
    ros::ServiceServer _srvStart;

};

#endif //#ifndef URSERVICEINTERFACE_HPP
