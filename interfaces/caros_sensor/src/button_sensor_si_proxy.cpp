/**/
#include <caros/button_sensor_si_proxy.h>

#include <boost/foreach.hpp>
#include <fstream>

using namespace rw::common;
using namespace std;
using namespace caros;

ButtonSensorSIProxy::ButtonSensorSIProxy(rw::common::Ptr<ros::NodeHandle> nhandle) :
    _nodeHnd(nhandle)
{
}

ButtonSensorSIProxy::ButtonSensorSIProxy(const std::string& name) :
    _nodeHnd(ownedPtr(new ros::NodeHandle(name)))
{
}

bool ButtonSensorSIProxy::configureProxy()
{
  _buttonSensorState = _nodeHnd->subscribe(_nodeHnd->getNamespace() + "/buttons", 1,
                                           &ButtonSensorSIProxy::handleButtonSensorState, this);
  return true;
}

ButtonSensorSIProxy::~ButtonSensorSIProxy()
{
}

void ButtonSensorSIProxy::handleButtonSensorState(const caros_sensor_msgs::button_sensor_state& state)
{
  boost::mutex::scoped_lock lock(_mutex);
  _stamp = state.header.stamp;
  _buttons.resize(state.digital.size() + state.analog.size());

  for (size_t i = 0; i < state.digital.size(); i++)
  {
    ButtonData &pdata = _buttons[i];
    pdata.button = state.digital[i];
    pdata.id = state.digital_ids[i];
  }
  for (size_t j = 0; j < state.analog.size(); j++)
  {
    ButtonData &pdata = _buttons[state.digital.size() + j];
    pdata.button = state.analog[j];
    pdata.id = state.analog_ids[j];
  }
}

std::vector<ButtonSensorSIProxy::ButtonData> ButtonSensorSIProxy::getButtons()
{
  boost::mutex::scoped_lock lock(_mutex);
  return _buttons;
}

ros::Time ButtonSensorSIProxy::getTimeStamp()
{
  boost::mutex::scoped_lock lock(_mutex);
  return _stamp;
}

