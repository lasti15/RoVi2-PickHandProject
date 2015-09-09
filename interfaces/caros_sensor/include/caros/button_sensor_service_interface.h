#ifndef CAROS_BUTTONSENSORSERVICEINTERFACE_H
#define CAROS_BUTTONSENSORSERVICEINTERFACE_H

#include <ros/ros.h>

#include <string>
#include <vector>
#include <utility>

#define BUTTON_SENSOR_BUTTONS_PUBLISHER_QUEUE_SIZE 1
#define BUTTON_SENSOR_SERVICE_INTERFACE_SUB_NAMESPACE "caros_button_sensor_service_interface"

namespace caros
{
/**
 * @brief describe the minimum interface of a button sensing device.
 */
class ButtonSensorServiceInterface
{
 public:
  //! constructor
  ButtonSensorServiceInterface(const ros::NodeHandle& nodehandle);

  virtual ~ButtonSensorServiceInterface();

 protected:
  //! initialize ros interface
  bool configureInterface();

  //! publish button states read by sensor
  void publishButtons(const std::vector<std::pair<std::string, bool>>& digital_buttons,
                      const std::vector<std::pair<std::string, bool>>& analog_buttons);

 private:
  ButtonSensorServiceInterface(){};

  ros::NodeHandle nodehandle_;
  ros::Publisher button_publisher_;
};
}

#endif
