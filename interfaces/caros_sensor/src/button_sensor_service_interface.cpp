#include <caros/button_sensor_service_interface.h>
#include <caros_sensor_msgs/ButtonSensorState.h>

using namespace caros;

ButtonSensorServiceInterface::ButtonSensorServiceInterface(const ros::NodeHandle& nodehandle) : nodehandle_(nodehandle)
{
}

ButtonSensorServiceInterface::~ButtonSensorServiceInterface()
{
  /* Nothing specific to do */
}

bool ButtonSensorServiceInterface::configureInterface()
{
  button_pub_ = nodehandle_.advertise<caros_sensor_msgs::ButtonSensorState>("buttons",
                                                                            BUTTON_SENSOR_BUTTONS_PUBLISHER_QUEUE_SIZE);
  return true;
}

void ButtonSensorServiceInterface::publishButtons(const std::vector<std::pair<std::string, bool>>& digital_buttons,
                                                  const std::vector<std::pair<std::string, bool>>& analog_buttons)
{
  caros_sensor_msgs::ButtonSensorState button_state;
  button_state.digital_ids.resize(digital_buttons.size());
  button_state.digital.resize(digital_buttons.size());
  button_state.analog_ids.resize(analog_buttons.size());
  button_state.analog.resize(analog_buttons.size());

  for (size_t i = 0; i < digital_buttons.size(); i++)
  {
    button_state.digital_ids[i] = digital_buttons[i].first;
    button_state.digital[i] = digital_buttons[i].second;
  }

  for (size_t i = 0; i < analog_buttons.size(); i++)
  {
    button_state.analog_ids[i] = analog_buttons[i].first;
    button_state.analog[i] = analog_buttons[i].second;
  }

  button_pub_.publish(button_state);
}
