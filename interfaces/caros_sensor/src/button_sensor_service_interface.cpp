/**/
#include <caros/button_sensor_service_interface.h>
#include <caros_sensor_msgs/ButtonSensorState.h>

#include <caros/common.h>

using namespace rw::common;
using namespace caros;

ButtonSensorServiceInterface::ButtonSensorServiceInterface(const ros::NodeHandle& nodeHnd) :
    node_hnd_(nodeHnd)
{
}

bool ButtonSensorServiceInterface::configureInterface(){
  button_pub_ = node_hnd_.advertise < caros_sensor_msgs::ButtonSensorState > ("buttons", 5);
  return true;
}

void ButtonSensorServiceInterface::publishButtons(const std::vector<std::pair<std::string,bool> >& digital_buttons,
                                                  const std::vector<std::pair<std::string,bool> >& analog_buttons)
{

  caros_sensor_msgs::ButtonSensorState pstate;
  pstate.digital_ids.resize(digital_buttons.size());
  pstate.digital.resize(digital_buttons.size());
  pstate.analog_ids.resize(analog_buttons.size());
  pstate.analog.resize(analog_buttons.size());

  for (size_t i = 0; i < digital_buttons.size(); i++)
  {
    pstate.digital_ids[i] = digital_buttons[i].first;
    pstate.digital[i] = digital_buttons[i].second;
  }


  for (size_t i = 0; i < analog_buttons.size(); i++)
  {
    pstate.analog_ids[i] = analog_buttons[i].first;
    pstate.analog[i] = analog_buttons[i].second;
  }

  button_pub_.publish(pstate);
}

