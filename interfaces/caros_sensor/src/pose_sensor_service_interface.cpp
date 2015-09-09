#include <caros/pose_sensor_service_interface.h>
#include <caros_sensor_msgs/PoseSensorState.h>

#include <caros/common_robwork.h>

using namespace rw::common;
using namespace caros;

PoseSensorServiceInterface::PoseSensorServiceInterface(const ros::NodeHandle& nodehandle) : nodehandle_(nodehandle)
{
}

bool PoseSensorServiceInterface::configureInterface()
{
  pose_pub_ = nodehandle_.advertise<caros_sensor_msgs::PoseSensorState>("poses", POSE_SENSOR_POSE_PUBLISHER_QUEUE_SIZE);
  return true;
}

void PoseSensorServiceInterface::publishPoses(const std::vector<rw::math::Transform3D<>>& poses,
                                              const std::vector<int>& ids, const std::vector<float>& qualities)
{
  caros_sensor_msgs::PoseSensorState pstate;
  pstate.poses.resize(poses.size());
  pstate.ids.resize(poses.size());
  pstate.qualities.resize(poses.size());

  for (size_t i = 0; i < poses.size(); i++)
  {
    pstate.poses[i] = caros::toRos(poses[i]);
    if (ids.size() <= i)
      pstate.ids[i] = i;
    else
      pstate.ids[i] = ids[i];

    if (qualities.size() <= i)
      pstate.qualities[i] = i;
    else
      pstate.qualities[i] = qualities[i];
  }

  pose_pub_.publish(pstate);
}
