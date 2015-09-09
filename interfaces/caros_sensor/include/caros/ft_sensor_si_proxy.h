#ifndef CAROS_FTSENSORSIPROXY_HPP
#define CAROS_FTSENSORSIPROXY_HPP

#include <geometry_msgs/WrenchStamped.h>

#include <rw/math.hpp>

#include <ros/ros.h>

#include <memory>
#include <mutex>

namespace caros
{
/**
 * @brief this class implements a cpp proxy to control and read data from
 * a FTSensorServiceInterface.
 *
 */
class FTSensorSIProxy
{
 public:
  //! pointer type
  typedef std::shared_ptr<FTSensorSIProxy> Ptr;

  /**
   * @brief Constructor
   * @param[in] nodehandle
   * @param[in] devname The name of the node
   * @param[in] use_persistent_connections Define usage of persistent connections
   */
  FTSensorSIProxy(ros::NodeHandle nodehandle, const std::string& devname, const bool use_persistent_connections = true);

  //! destructor
  virtual ~FTSensorSIProxy();

  //! get current state
  rw::math::Wrench6D<> getWrench();

  //! get time stamp of current reading
  ros::Time getTimeStamp();

 protected:
  ros::NodeHandle nodehandle_;

  // states
  ros::Subscriber ft_state_sub_;

 private:
  std::mutex mutex_;

  // state variables
  rw::math::Wrench6D<> wrench_;

  void handleFTState(const geometry_msgs::WrenchStamped& state);

  geometry_msgs::WrenchStamped ft_state_;
};
}

#endif
