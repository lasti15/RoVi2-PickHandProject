/**/
#ifndef SERIALDEVICESERVICEINTERFACE_HPP
#define SERIALDEVICESERVICEINTERFACE_HPP

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/VelocityScrew6D.hpp>
#include <rw/common/Ptr.hpp>

#include "marvin_common/RobotState.h"
#include "marvin_common/MarvinUtils.hpp"
#include <marvin_common/SerialDeviceMoveLin.h>
#include <marvin_common/SerialDeviceMovePTP.h>
#include <marvin_common/SerialDeviceMovePTP_T.h>
#include <marvin_common/SerialDeviceMoveVelQ.h>
#include <marvin_common/SerialDeviceMoveVelT.h>
#include <marvin_common/SerialDeviceMoveLinFC.h>
#include <marvin_common/ConfigBool.h>
#include <marvin_common/Stop.h>
#include <marvin_common/Pause.h>
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

protected:

	//! initialize services in the node handle
	void initNodeHandle();


	//! @brief move robot in a linear Cartesian path
	virtual bool moveLin(marvin_common::SerialDeviceMoveLin::Request& request,
				  marvin_common::SerialDeviceMoveLin::Response& response){return false;};

	//! @brief move robot from point to point
	virtual bool movePTP(marvin_common::SerialDeviceMovePTP::Request& request,
				  marvin_common::SerialDeviceMovePTP::Response& response){return false;};

	//! @brief move robot from point to point but using a pose as target (require invkin)
	virtual bool movePTP_T(marvin_common::SerialDeviceMovePTP_T::Request& request,
				  marvin_common::SerialDeviceMovePTP_T::Response& response){return false;};

	//! @brief move robot in a servoing fasion specifying joint velocity targets
	virtual bool moveVelQ(marvin_common::SerialDeviceMoveVelQ::Request& request,
				   marvin_common::SerialDeviceMoveVelQ::Response& response){return false;};

	//! @brief move robot in a servoing fasion specifying a velocity screw in tool coordinates
	virtual bool moveVelT(marvin_common::SerialDeviceMoveVelT::Request& request,
				   marvin_common::SerialDeviceMoveVelT::Response& response){return false;};

	//! @brief move robot in a servoing fasion specifying joint velocity targets
	virtual bool servoQ(marvin_common::SerialDeviceMovePTP::Request& request,
				   marvin_common::SerialDeviceMovePTP::Response& response){return false;};

	//! @brief move robot in a servoing fasion specifying a velocity screw in tool coordinates
	virtual bool servoT(marvin_common::SerialDeviceMovePTP_T::Request& request,
				   marvin_common::SerialDeviceMovePTP_T::Response& response){return false;};


	//! move robot with a hybrid position/force control
	virtual bool moveLinFC(marvin_common::SerialDeviceMoveLinFC::Request& request,
				     marvin_common::SerialDeviceMoveLinFC::Response& response){return false;};

	//! hard stop the robot,
	virtual bool start(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){return false;};

	//! hard stop the robot,
	virtual bool stop(marvin_common::Stop::Request& request,
						marvin_common::Stop::Response& response){return false;};

	//! pause the robot, should be able to continue trajectory
	virtual bool pause(marvin_common::Pause::Request& request,
						 marvin_common::Pause::Response& response){return false;};

	//! enable safe mode, so that robot stops when collisions are detected
	virtual bool setSafeModeEnabled(marvin_common::ConfigBool::Request& request,
									    marvin_common::ConfigBool::Request& response){return false;};

	//! publish robot state
	void publish(const marvin_common::RobotState& state);

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



	// the interface that a robotarm should be able to extend
	/*
	// linear movements
	void lin( const rw::math::Transform3D<>& ); // blend, speed

	//void moveL( const rw::math::Transform3D<>& , const rw::math::VelocityScrew6D<>& );

	// with blends in percentage [0-1]
	void lin( const std::vector<rw::math::Transform3D<> >& targets, const std::vector<float>& blendValues);

	// trajectory movements
	void move( const rw::trajectory::Transform3DTrajectory& traj );

	// joint movements
	void pTp( const rw::math::Q& target ); // blend, speed,
	void pTp( const rw::math::Transform3D<>& target ); // blend, speed,  default closest solution or elbow up/down

	// with blends
	void pTp( const std::vector<rw::math::Q>& targets, const std::vector<float>& blendValues );
	// trajectory movements
	void pTp( const rw::trajectory::QTrajectory& traj );
	// force/torque based control
	//void moveQ( const rw::math::Q& );
	// hybrid position/force based control, setting wrench and selection matrix

	void linFC( const rw::math::Transform3D<>& target,
			      const rw::math::Wrench<>& ,
			    	int selection[],
			    	rw::math::Transform3D<> offset,
			    	const std::string& name); // blend, speed
    */



    /*ros::Duration _syncTimeOffset;
    ros::Duration _driverTimeOffset;
*/
};

#endif //#ifndef URSERVICEINTERFACE_HPP
