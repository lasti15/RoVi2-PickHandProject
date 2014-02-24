/*
 * FTSensorSIProxy.cpp
 *
 *  Created on: 15/05/2013
 *      Author: thomas
 */

#include <marvin_common_rw/FTSensorSIProxy.hpp>
#include <marvin_common_rw/RwRos.hpp>

FTSensorSIProxy::FTSensorSIProxy(rw::common::Ptr<ros::NodeHandle> nhandle, const std::string& devname):
	_nodeHnd(nhandle)
{
	_ftState = _nodeHnd->subscribe("wrench", 1, &FTSensorSIProxy::handleFTState, this);
}

FTSensorSIProxy::~FTSensorSIProxy() {
}

void FTSensorSIProxy::handleFTState(const marvin_common::WrenchData& state) {
	boost::mutex::scoped_lock lock(_mutex);
	_wrench = RwRos::toRw(state.wrench);
	_pFTState = state;
}

rw::math::Wrench6D<> FTSensorSIProxy::getWrench() {
	boost::mutex::scoped_lock lock(_mutex);
	return _wrench;
}

ros::Time FTSensorSIProxy::getTimeStamp() {
	boost::mutex::scoped_lock lock(_mutex);
	return _pFTState.header.stamp;
}
