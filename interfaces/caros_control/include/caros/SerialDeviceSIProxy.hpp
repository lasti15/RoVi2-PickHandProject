#ifndef SerialDeviceSIProxy_HPP_
#define SerialDeviceSIProxy_HPP_

#include <caros_control_msgs/RobotState.h>

//#include <rw/math.hpp>
//#include <rw/trajectory/Path.hpp>

//#include <boost/thread.hpp>

#include <ros/ros.h>

//#include <queue>

namespace caros {
/**
 * @brief this class implements a cpp proxy to control and read data from
 * a SerialDeviceServiceInterface.
 *
 */
    class SerialDeviceSIProxy {

    public:
        /**
         * @brief Constructor
         * @param[in] nodehandle
         * @param[in] devname The name of the CAROS serialdevice node
         */
	SerialDeviceSIProxy(ros::NodeHandle nodehandle, const std::string& devname);

	//! destructor
	virtual ~SerialDeviceSIProxy();

	//! @brief move robot in a linear Cartesian path
	bool moveLin(const rw::math::Transform3D<>& target, float speed = 100, float blend = 0);

	//! @brief move robot from point to point
	bool movePTP(const rw::math::Q& target, float speed = 100, float blend = 0);

	//! @brief move robot from point to point but using a pose as target (require invkin)
	bool movePTP_T(const rw::math::Transform3D<>& target, float speed = 100, float blend = 0);

	//! @brief move robot in a servoing fasion
	bool moveVelQ(const rw::math::Q& target);

	bool moveVelT(const rw::math::VelocityScrew6D<>& target);

	//! @brief move robot in a servoing fasion
	bool moveServoQ(const rw::math::Q& target, float speed = 100);

	bool moveServoT(const rw::math::Transform3D<>& target, float speed = 100);

	//! move robot with a hybrid position/force control
	bool moveLinFC(const rw::math::Transform3D<>& target,
                               rw::math::Wrench6D<>& wtarget,
                               float selection[6],
                               std::string refframe,
                               rw::math::Transform3D<> offset,
                               float speed = 100,
                               float blend = 0);

	//! hard stop the robot,
	bool stop();

	//! pause the robot, should be able to continue trajectory
	bool pause();

	//! enable safe mode, so that robot stops when collisions are detected
	bool setSafeModeEnabled(bool enable);

	rw::math::Q getQ();
	rw::math::Q getQd();

	ros::Time getTimeStamp();

	bool isMoving();

    protected:
	ros::NodeHandle _nodeHnd;
	ros::ServiceClient _servoService;

	// services
	ros::ServiceClient _srvStop;
	ros::ServiceClient _srvStart;
	ros::ServiceClient _srvPause;

	ros::ServiceClient _srvMovePTP;
	ros::ServiceClient _srvMovePTP_T;
	ros::ServiceClient _srvMoveLin;
	ros::ServiceClient _srvMoveLinFC;
	ros::ServiceClient _srvMoveVelQ;
	ros::ServiceClient _srvMoveVelT;
	ros::ServiceClient _srvServoQ;
	ros::ServiceClient _srvServoT;

	// states
	ros::Subscriber _subRobotState;

    private:
	boost::mutex _mutex;

	// state variables
	rw::math::Q _q, _dq;
	bool _isRunning;
	bool _stopRobot;

	void handleRobotState(const caros_control_msgs::RobotState& state);
	caros_control_msgs::RobotState _pRobotState;
	double _zeroTime;
    };

}

#endif //end include guard
