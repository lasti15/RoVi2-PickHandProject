/**/
#ifndef CAROS_BUTTONSENSORSERVICEINTERFACE_H
#define CAROS_BUTTONSENSORSERVICEINTERFACE_H

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>

#include <ros/ros.h>
#include <string>

namespace caros
{

/**
 * @brief describe the minimum interface of a button sensing device.
 */
class ButtonSensorServiceInterface
{
public:
  ButtonSensorServiceInterface(const std::string& service_name);

  //! constructor
  ButtonSensorServiceInterface(const ros::NodeHandle& nodehandle);

protected:
  bool configureInterface();

  //! publish poses read by sensor
  void publishButtons(const std::vector<std::pair<std::string,bool> >& digital_buttons,
                      const std::vector<std::pair<std::string,bool> >& analog_buttons);


protected:
  ros::NodeHandle node_hnd_;
private:
  ros::Publisher button_pub_;

};

}

#endif
