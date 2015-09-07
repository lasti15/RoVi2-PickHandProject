#ifndef CAROS_BUTTONSESORSIPROXY_H
#define CAROS_BUTTONSESORSIPROXY_H

#include <caros_sensor_msgs/button_sensor_state.h>

#include <memory>
#include <mutex>

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
  typedef std::shared_ptr<ButtonSensorSIProxy> Ptr;

  //! constructor
  ButtonSensorSIProxy(ros::NodeHandle nhandle);

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
  ros::NodeHandle _nodeHnd;

  // states
  ros::Subscriber _buttonSensorState;

 private:
  std::mutex _mutex;

  // state variables
  /* Notes:
   * std::vector could potentially, depending on the number of elements, be an ineffective container for looking up
   * specific button IDs, since the list has to be traversed until the ID is found.
   */
  std::vector<ButtonData> _buttons;
  ros::Time _stamp;
};
}
#endif  // end include guard
