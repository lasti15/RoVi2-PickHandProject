/**/
#include <marvin_common_rw/PoseSensorSIProxy.hpp>

#include <fstream>

#include <marvin_common/URServoQ.h>
#include <marvin_common_rw/RwRos.hpp>


#include <rw/common/Ptr.hpp>

#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace std;

PoseSensorSIProxy::PoseSensorSIProxy(rw::common::Ptr<ros::NodeHandle> nhandle, const std::string& devname):
		_nodeHnd(nhandle)
{
	// set up everything to control the robot
	// services
	_srvStart = _nodeHnd->serviceClient<std_srvs::Empty> (devname + "/start");
	_srvStop = _nodeHnd->serviceClient<marvin_common::Stop> (devname + "/stop");
	_srvPause = _nodeHnd->serviceClient<marvin_common::Pause> (devname + "/pause");

	// states
	_poseSensorState = _nodeHnd->subscribe(devname + "/poses", 1, &PoseSensorSIProxy::handlePoseSensorState, this);

}

PoseSensorSIProxy::~PoseSensorSIProxy() {
}

//! hard stop the robot,
bool PoseSensorSIProxy::start(){
	std_srvs::Empty srv;
	return _srvStop.call(srv);
}

//! hard stop the robot,
bool PoseSensorSIProxy::stop(){
	marvin_common::Stop srv;
	return _srvStop.call(srv);
}

//! pause the robot, should be able to continue trajectory
bool PoseSensorSIProxy::pause(){
	marvin_common::Pause srv;
	return _srvPause.call(srv);
}

void PoseSensorSIProxy::handlePoseSensorState(const marvin_common::PoseSensorState& state)
{
	boost::mutex::scoped_lock lock(_mutex);
	_poses.resize(state.poses.size());
	_stamp = state.header.stamp;
	for(int i=0;i<state.poses.size();i++){
		PoseData &pdata = _poses[i];
		pdata.pose = RwRos::toRw(state.poses[i]);
		pdata.id = state.ids[i];
		pdata.quality = state.qualities[i];
		pdata.stamp = state.header.stamp;
		pdata.frame = state.header.frame_id;
	}
}


std::vector<PoseSensorSIProxy::PoseData> PoseSensorSIProxy::getPoses() {
	boost::mutex::scoped_lock lock(_mutex);
	return _poses;
}

ros::Time PoseSensorSIProxy::getTimeStamp() {
	boost::mutex::scoped_lock lock(_mutex);
	return _stamp;
}

