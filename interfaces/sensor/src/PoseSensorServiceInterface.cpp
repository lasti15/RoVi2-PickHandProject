/**/
#include <caros/PoseSensorServiceInterface.hpp>
#include <caros_sensor/PoseSensorState.h>

using namespace rw::common;

PoseSensorServiceInterface::PoseSensorServiceInterface(){

}

PoseSensorServiceInterface::PoseSensorServiceInterface(const std::string& service_name)
{
	_nodeHnd = ownedPtr( new ros::NodeHandle(service_name) );
    initNodeHandler();
}

PoseSensorServiceInterface::PoseSensorServiceInterface(rw::common::Ptr<ros::NodeHandle> nodeHnd):
		_nodeHnd(nodeHnd)
{
	initNodeHandler();
}

void PoseSensorServiceInterface::initPoseSensorSI(const std::string& service_name){
	_nodeHnd = ownedPtr( new ros::NodeHandle(service_name) );
    initNodeHandler();
}

void PoseSensorServiceInterface::initPoseSensorSI(ros::NodeHandle& nodeHnd){
	_nodeHnd = rw::common::ownedPtr( new ros::NodeHandle(nodeHnd) );
    initNodeHandler();
}

void PoseSensorServiceInterface::initNodeHandler(){

	_posePublisher = _nodeHnd->advertise<PoseSensorState>("poses", 5);

    _srvStart = _nodeHnd->advertiseService("start", &PoseSensorServiceInterface::start, this);
    _srvStop = _nodeHnd->advertiseService("stop", &PoseSensorServiceInterface::stop, this);
    _srvPause = _nodeHnd->advertiseService("pause", &PoseSensorServiceInterface::pause, this);
}

void PoseSensorServiceInterface::publishPoseSensorSI(
		   const std::vector<rw::math::Transform3D<> >& poses,
		   const std::vector<int>& ids,
		   const std::vector<float>& qualities)
{
	PoseSensorState pstate;
	pstate.poses.resize( poses.size() );
	pstate.ids.resize( poses.size() );
	pstate.qualities.resize( poses.size() );

	for(int i=0;i<poses.size();i++){
		pstate.poses[i] = RwRos::toRos(poses[i]);
		if(ids.size()<=i)
			pstate.ids[i] = i;
		else
			pstate.ids[i] = ids[i];

		if(qualities.size()<=i)
			pstate.qualities[i] = i;
		else
			pstate.qualities[i] = qualities[i];


	}

	_posePublisher.publish( pstate );
}
