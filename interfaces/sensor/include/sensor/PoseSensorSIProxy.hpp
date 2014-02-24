/**/
#ifndef PoseSensorSIProxy_HPP_
#define PoseSensorSIProxy_HPP_

#include <marvin_common/RobotState.h>
#include <marvin_common/PoseSensorState.h>
#include <marvin_common_rw/PoseSensorServiceInterface.hpp>

#include <rw/common/Ptr.hpp>
#include <rw/math.hpp>
#include <rw/trajectory/Path.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>

#include <queue>


/**
 * @brief this class implements a cpp proxy to control and read data from
 * a PoseSensorServiceInterface.
 *
 */
class PoseSensorSIProxy {

public:
	typedef rw::common::Ptr<PoseSensorSIProxy> Ptr;

	//! constructor - create with device name
	PoseSensorSIProxy(rw::common::Ptr<ros::NodeHandle> nhandle, const std::string& devname);

	//! destructor
	virtual ~PoseSensorSIProxy();

	//! hard stop the robot,
	bool start();

	//! hard stop the robot,
	bool stop();

	//! pause the robot, should be able to continue trajectory
	bool pause();


	struct PoseData {
		rw::math::Transform3D<> pose;
		int id;
		float quality;
		ros::Time stamp;
		std::string frame;
	};

	std::vector<PoseData> getPoses();

	ros::Time getTimeStamp();


	void handlePoseSensorState(const marvin_common::PoseSensorState& state);

protected:
	rw::common::Ptr<ros::NodeHandle> _nodeHnd;
	// services
	ros::ServiceClient _srvStart;
	ros::ServiceClient _srvStop;
	ros::ServiceClient _srvPause;

	// states
	ros::Subscriber _poseSensorState;


private:
	boost::mutex _mutex;

	// state variables
	std::vector<PoseData> _poses;
	ros::Time _stamp;
	double _zeroTime;
};

#endif //end include guard
