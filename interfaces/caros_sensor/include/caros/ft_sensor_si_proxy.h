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

  //! constructor
  FTSensorSIProxy(ros::NodeHandle nhandle);

  //! constructor
  FTSensorSIProxy(const std::string& devname);

  //! destructor
  virtual ~FTSensorSIProxy();

  //! get current state
  rw::math::Wrench6D<> getWrench();

  //! get time stamp of current reading
  ros::Time getTimeStamp();

 protected:
  ros::NodeHandle _nodeHnd;

  // states
  ros::Subscriber _ftState;

 private:
  std::mutex _mutex;

  // state variables
  rw::math::Wrench6D<> _wrench;

  void handleFTState(const geometry_msgs::WrenchStamped& state);

  geometry_msgs::WrenchStamped _pFTState;
};
}

#endif
