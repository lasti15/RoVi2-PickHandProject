#include <caros/button_sensor_si_proxy.h>
using namespace std;
using namespace caros;

ButtonSensorSIProxy::ButtonSensorSIProxy(ros::NodeHandle nodehandle, const std::string& devname,
                                         const bool usePersistentConnections)
    : nodeHnd_(nodehandle)
{
  buttonSensorState_ =
      nodeHnd_.subscribe(nodeHnd_.getNamespace() + "/buttons", 1, &ButtonSensorSIProxy::handleButtonSensorState, this);
}

ButtonSensorSIProxy::~ButtonSensorSIProxy()
{
}

void ButtonSensorSIProxy::handleButtonSensorState(const caros_sensor_msgs::button_sensor_state& state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  stamp_ = state.header.stamp;
  buttons_.resize(state.digital.size() + state.analog.size());

  for (size_t i = 0; i < state.digital.size(); i++)
  {
    ButtonData& pdata = buttons_[i];
    pdata.button = state.digital[i];
    pdata.id = state.digital_ids[i];
    pdata.isAnalog = false;
    pdata.stamp = stamp_;
  }
  for (size_t j = 0; j < state.analog.size(); j++)
  {
    ButtonData& pdata = buttons_[state.digital.size() + j];
    pdata.button = state.analog[j];
    pdata.id = state.analog_ids[j];
    pdata.isAnalog = true;
    pdata.stamp = stamp_;
  }
}

std::vector<ButtonSensorSIProxy::ButtonData> ButtonSensorSIProxy::getButtons()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return buttons_;
}

ros::Time ButtonSensorSIProxy::getTimeStamp()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stamp_;
}
