/**/
#include <caros/ButtonSensorSIProxy.hpp>

#include <caros/common.hpp>
#include <rw/common/Ptr.hpp>
#include <boost/foreach.hpp>
#include <fstream>

using namespace rw::common;
using namespace rw::math;
using namespace std;

ButtonSensorSIProxy::ButtonSensorSIProxy(rw::common::Ptr<ros::NodeHandle> nhandle, const std::string& devname):
		_nodeHnd(nhandle)
{
	// set up everything to control the robot
	// services
	_srvStart = _nodeHnd->serviceClient<std_srvs::Empty> (devname + "/start");
	_srvStop = _nodeHnd->serviceClient<marvin_common::Stop> (devname + "/stop");
	_srvPause = _nodeHnd->serviceClient<marvin_common::Pause> (devname + "/pause");

	// states
	_buttonSensorState = _nodeHnd->subscribe(devname + "/buttons", 1, &ButtonSensorSIProxy::handleButtonSensorState, this);

}

ButtonSensorSIProxy::~ButtonSensorSIProxy() {
}

//! hard stop the robot,
bool ButtonSensorSIProxy::start(){
	std_srvs::Empty srv;
	return _srvStop.call(srv);
}

//! hard stop the robot,
bool ButtonSensorSIProxy::stop(){
	marvin_common::Stop srv;
	return _srvStop.call(srv);
}

//! pause the robot, should be able to continue trajectory
bool ButtonSensorSIProxy::pause(){
	marvin_common::Pause srv;
	return _srvPause.call(srv);
}

void ButtonSensorSIProxy::handleButtonSensorState(const marvin_common::ButtonSensorState& state)
{
	boost::mutex::scoped_lock lock(_mutex);
	_buttons.resize(state.digital.size() + state.analog.size());

	for(int i=0;i<state.digital.size();i++){
		ButtonData &pdata = _buttons[i];
		pdata.button = state.digital[i];
		pdata.id = state.digital_ids[i];
	}
	for(int j=0;j<state.analog.size();j++){
		ButtonData &pdata = _buttons[state.digital.size()+j];
		pdata.button = state.analog[j];
		pdata.id = state.analog_ids[j];
	}
}


std::vector<ButtonSensorSIProxy::ButtonData> ButtonSensorSIProxy::getButtons() {
	boost::mutex::scoped_lock lock(_mutex);
	return _buttons;
}

ros::Time ButtonSensorSIProxy::getTimeStamp() {
	boost::mutex::scoped_lock lock(_mutex);
	return _stamp;
}

