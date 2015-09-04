/*
 * FTSensorSIProxy.cpp
 *
 *  Created on: 15/05/2013
 *      Author: thomas
 */

#include <caros/common_robwork.h>

#include <caros/ft_sensor_si_proxy.h>

using namespace caros;

FTSensorSIProxy::FTSensorSIProxy(ros::NodeHandle nhandle) :
    _nodeHnd(nhandle)
{
  _ftState = _nodeHnd.subscribe("wrench", 1, &FTSensorSIProxy::handleFTState, this);
}

FTSensorSIProxy::FTSensorSIProxy(const std::string& name) :
    _nodeHnd(name)
{
  _ftState = _nodeHnd.subscribe("wrench", 1, &FTSensorSIProxy::handleFTState, this);
}

FTSensorSIProxy::~FTSensorSIProxy()
{
}

void FTSensorSIProxy::handleFTState(const geometry_msgs::WrenchStamped& state)
{
  boost::mutex::scoped_lock lock(_mutex);
  _wrench = caros::toRw(state.wrench);
  _pFTState = state;
}

rw::math::Wrench6D<> FTSensorSIProxy::getWrench()
{
  boost::mutex::scoped_lock lock(_mutex);
  return _wrench;
}

ros::Time FTSensorSIProxy::getTimeStamp()
{
  boost::mutex::scoped_lock lock(_mutex);
  return _pFTState.header.stamp;
}
