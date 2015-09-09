#include <caros/button_sensor_service_interface.h>
#include <caros_sensor_msgs/ButtonSensorState.h>

using namespace caros;

ButtonSensorServiceInterface::ButtonSensorServiceInterface(const ros::NodeHandle& nodeHnd) : node_hnd_(nodeHnd)
{
}

ButtonSensorServiceInterface::~ButtonSensorServiceInterface()
{
  /* Nothing specific to do */
}

bool ButtonSensorServiceInterface::configureInterface()
{
  button_pub_ = node_hnd_.advertise<caros_sensor_msgs::ButtonSensorState>("buttons",
                                                                            BUTTON_SENSOR_BUTTONS_PUBLISHER_QUEUE_SIZE);
  return true;
}

void ButtonSensorServiceInterface::publishButtons(const std::vector<std::pair<std::string, bool>>& digitalbuttons_,
                                                  const std::vector<std::pair<std::string, bool>>& analogbuttons_)
{
  caros_sensor_msgs::ButtonSensorState pstate;
  pstate.digital_ids.resize(digitalbuttons_.size());
  pstate.digital.resize(digitalbuttons_.size());
  pstate.analog_ids.resize(analogbuttons_.size());
  pstate.analog.resize(analogbuttons_.size());

  for (size_t i = 0; i < digitalbuttons_.size(); i++)
  {
    pstate.digital_ids[i] = digitalbuttons_[i].first;
    pstate.digital[i] = digitalbuttons_[i].second;
  }

  for (size_t i = 0; i < analogbuttons_.size(); i++)
  {
    pstate.analog_ids[i] = analogbuttons_[i].first;
    pstate.analog[i] = analogbuttons_[i].second;
  }

  button_pub_.publish(pstate);
}
