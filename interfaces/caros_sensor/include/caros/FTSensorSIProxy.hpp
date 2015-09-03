/*
 * FTSensorSIProxy.hpp
 *
 *  Created on: 15/05/2013
 *      Author: thomas
 */

#ifndef CAROS_FTSENSORSIPROXY_HPP
#define CAROS_FTSENSORSIPROXY_HPP

#include <geometry_msgs/WrenchStamped.h>

#include <rw/common/Ptr.hpp>
#include <rw/math.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>

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
  typedef rw::common::Ptr<FTSensorSIProxy> Ptr;

  //! constructor
  FTSensorSIProxy(rw::common::Ptr<ros::NodeHandle> nhandle);

  //! constructor
  FTSensorSIProxy(const std::string& devname);

  //! destructor
  virtual ~FTSensorSIProxy();

  //! get current state
  rw::math::Wrench6D<> getWrench();

  //! get time stamp of current reading
  ros::Time getTimeStamp();

protected:
  rw::common::Ptr<ros::NodeHandle> _nodeHnd;

  // states
  ros::Subscriber _ftState;

private:
  boost::mutex _mutex;

  // state variables
  rw::math::Wrench6D<> _wrench;

  void handleFTState(const geometry_msgs::WrenchStamped& state);
  
  geometry_msgs::WrenchStamped _pFTState;
};

}

#endif
