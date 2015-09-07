#ifndef CAROS_POSESENSORSIPROXY_HPP
#define CAROS_POSESENSORSIPROXY_HPP

#include <caros_sensor_msgs/pose_sensor_state.h>

#include <rw/math.hpp>

#include <ros/ros.h>

#include <memory>
#include <mutex>
#include <vector>
#include <string>

namespace caros
{
/**
 * @brief this class implements a cpp proxy to control and read data from
 * a PoseSensorServiceInterface.
 */
class PoseSensorSIProxy
{
 public:
  //! pointer type
  typedef std::shared_ptr<PoseSensorSIProxy> Ptr;

  //! constructor
  PoseSensorSIProxy(const ros::NodeHandle& nhandle);

  //! constructor
  PoseSensorSIProxy(const std::string& devname);

  //! destructor
  virtual ~PoseSensorSIProxy();

  //! pose data structure
  struct PoseData
  {
    rw::math::Transform3D<> pose;
    int id;
    float quality;
    ros::Time stamp;
    std::string frame;
  };

  //! get current pose state
  std::vector<PoseData> getPoses();

  //! get time stamp of current pose state
  ros::Time getTimeStamp();

 protected:
  void configureProxy();

  void handlePoseSensorState(const caros_sensor_msgs::pose_sensor_state& state);

 protected:
  ros::NodeHandle node_hnd_;

  // states
  ros::Subscriber _poseSensorState;

 private:
  std::mutex _mutex;

  // state variables
  std::vector<PoseData> _poses;
  ros::Time _stamp;
};
}

#endif
