#ifndef CAROS_BUTTONSENSORSERVICEINTERFACE_H
#define CAROS_BUTTONSENSORSERVICEINTERFACE_H

#include <ros/ros.h>

#include <string>
#include <vector>
#include <utility>

#define BUTTON_SENSOR_BUTTONS_PUBLISHER_QUEUE_SIZE 1

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
  //! setup the node's output services
  bool configureInterface();

  //! publish button states read by sensor
  void publishButtons(const std::vector<std::pair<std::string, bool>>& digitalbuttons_,
                      const std::vector<std::pair<std::string, bool>>& analogbuttons_);

 protected:
  ros::NodeHandle node_hnd_;

 private:
  ros::Publisher button_pub_;
};
}

#endif
