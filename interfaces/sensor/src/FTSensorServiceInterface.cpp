/**/
#include <marvin_common_rw/FTSensorServiceInterface.hpp>

#include <marvin_common_rw/RwRos.hpp>

#include <marvin_common/WrenchData.h>
#include <rw/math/Wrench6D.hpp>

using namespace marvin_common;
using namespace rw::common;

FTSensorServiceInterface::FTSensorServiceInterface(const std::string& service_name)
{
    _nodeHnd = ownedPtr( new ros::NodeHandle(service_name) );

    _wrenchDataPublisher = _nodeHnd->advertise<marvin_common::WrenchData>("wrench", 5);
    //_robotStatePublisher = _nodeHnd.advertise<SDHState>("SDHState", 5);
    //_srvStop = _nodeHnd.advertiseService("stop", &SDHServiceInterface::stopHandle, this);
}


void FTSensorServiceInterface::publish(const rw::math::Wrench6D<>& wrench, const std::string& refframe)
{
	marvin_common::WrenchData wdata;
	wdata.header.frame_id = refframe;
	wdata.header.stamp = ros::Time::now();

	wdata.wrench.force.x = wrench(0);
	wdata.wrench.force.y = wrench(1);
	wdata.wrench.force.z = wrench(2);

	wdata.wrench.torque.x = wrench(3);
	wdata.wrench.torque.y = wrench(4);
	wdata.wrench.torque.z = wrench(5);

	_wrenchDataPublisher.publish( wdata );
}
