/**/
#ifndef CAROS_BUTTONSESORSIPROXY_H
#define CAROS_BUTTONSESORSIPROXY_H

#include <caros_sensor_msgs/button_sensor_state.h>

#include <rw/common/Ptr.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>

namespace caros
{

/**
 * @brief this class implements a cpp proxy to control and read data from
 * a ButtonSensorServiceInterface.
 *
 */
class ButtonSensorSIProxy
{

public:
  //! pointer type
  typedef rw::common::Ptr<ButtonSensorSIProxy> Ptr;

  //! constructor
  ButtonSensorSIProxy(rw::common::Ptr<ros::NodeHandle> nhandle);

  //! constructor
  ButtonSensorSIProxy(const std::string& devname);

  //! destructor
  virtual ~ButtonSensorSIProxy();

  //! button data structure
  struct ButtonData
  {
    float button;
    std::string id;
    bool isAnalog;
    ros::Time stamp;
  };

  //! get current button state
  std::vector<ButtonData> getButtons();

  //! get time stamp of current button state
  ros::Time getTimeStamp();

protected:
  bool configureProxy();
  
  void handleButtonSensorState(const caros_sensor_msgs::button_sensor_state& state);

protected:
  rw::common::Ptr<ros::NodeHandle> _nodeHnd;

  // states
  ros::Subscriber _buttonSensorState;

private:
  boost::mutex _mutex;

  // state variables
  std::vector<ButtonData> _buttons;
  ros::Time _stamp;
};

}
#endif //end include guard
