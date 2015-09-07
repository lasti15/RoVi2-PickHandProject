/**/
#include <caros/pose_sensor_si_proxy.h>

#include <caros/common_robwork.h>

using namespace rw::common;
using namespace rw::math;
using namespace std;

using namespace caros;

PoseSensorSIProxy::PoseSensorSIProxy(const ros::NodeHandle& nhandle) : node_hnd_(nhandle)
{
}

PoseSensorSIProxy::PoseSensorSIProxy(const std::string& devname) : node_hnd_(devname)
{
}

PoseSensorSIProxy::~PoseSensorSIProxy()
{
}

void PoseSensorSIProxy::configureProxy()
{
  _poseSensorState =
      node_hnd_.subscribe(node_hnd_.getNamespace() + "/poses", 1, &PoseSensorSIProxy::handlePoseSensorState, this);
}

void PoseSensorSIProxy::handlePoseSensorState(const caros_sensor_msgs::pose_sensor_state& state)
{
  boost::mutex::scoped_lock lock(_mutex);
  _poses.resize(state.poses.size());
  _stamp = state.header.stamp;
  for (size_t i = 0; i < state.poses.size(); i++)
  {
    PoseData& pdata = _poses[i];
    pdata.pose = caros::toRw(state.poses[i]);
    pdata.id = state.ids[i];
    pdata.quality = state.qualities[i];
    pdata.stamp = state.header.stamp;
    pdata.frame = state.header.frame_id;
  }
}

std::vector<PoseSensorSIProxy::PoseData> PoseSensorSIProxy::getPoses()
{
  boost::mutex::scoped_lock lock(_mutex);
  return _poses;
}

ros::Time PoseSensorSIProxy::getTimeStamp()
{
  boost::mutex::scoped_lock lock(_mutex);
  return _stamp;
}
