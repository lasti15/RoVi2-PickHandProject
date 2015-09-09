#ifndef CAROS_POSESENSORSIPROXY_HPP
#define CAROS_POSESENSORSIPROXY_HPP

#include <caros_sensor_msgs/ButtonSensorState.h>
#include <caros_sensor_msgs/PoseSensorState.h>

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

  /**
   * @brief Constructor
   * @param[in] nodehandle
   * @param[in] devname The name of the node
   * @param[in] usePersistentConnections Define usage of persistent connections
   */
  PoseSensorSIProxy(ros::NodeHandle nodehandle, const std::string& devname, const bool usePersistentConnections = true);

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
  void handlePoseSensorState(const caros_sensor_msgs::PoseSensorState& state);

 protected:
  ros::NodeHandle node_hnd_;

  // states
  ros::Subscriber poseSensorState_;

 private:
  std::mutex mutex_;

  // state variables
  std::vector<PoseData> _poses;
  ros::Time stamp_;
};
}

#endif
