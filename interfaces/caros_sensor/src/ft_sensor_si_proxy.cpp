#include <caros/ft_sensor_si_proxy.h>

#include <caros/common_robwork.h>

using namespace caros;

FTSensorSIProxy::FTSensorSIProxy(ros::NodeHandle nodehandle, const std::string& devname,
                                 const bool usePersistentConnections)
    : nodeHnd_(nodehandle)
{
  _ftState = nodeHnd_.subscribe("wrench", 1, &FTSensorSIProxy::handleFTState, this);
}

FTSensorSIProxy::~FTSensorSIProxy()
{
}

void FTSensorSIProxy::handleFTState(const geometry_msgs::WrenchStamped& state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  wrench_ = caros::toRw(state.wrench);
  pFTState_ = state;
}

rw::math::Wrench6D<> FTSensorSIProxy::getWrench()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return wrench_;
}

ros::Time FTSensorSIProxy::getTimeStamp()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return pFTState_.header.stamp;
}
