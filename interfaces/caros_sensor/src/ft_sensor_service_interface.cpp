#include <caros/ft_sensor_service_interface.h>

#include <caros/common.h>
#include <geometry_msgs/WrenchStamped.h>

using namespace rw::common;
using namespace caros;

FTSensorServiceInterface::FTSensorServiceInterface(ros::NodeHandle nh)
{
  nodeHnd_ = nh;
  wrenchDataPublisher_ =
      nodeHnd_.advertise<geometry_msgs::WrenchStamped>("wrench", FT_SENSOR_WRENCH_PUBLISHER_QUEUE_SIZE);
}

void FTSensorServiceInterface::publish(const rw::math::Wrench6D<>& wrench, const std::string& refframe)
{
  geometry_msgs::WrenchStamped wdata;

  wdata.header.frame_id = refframe;
  wdata.header.stamp = ros::Time::now();

  wdata.wrench.force.x = wrench(0);
  wdata.wrench.force.y = wrench(1);
  wdata.wrench.force.z = wrench(2);

  wdata.wrench.torque.x = wrench(3);
  wdata.wrench.torque.y = wrench(4);
  wdata.wrench.torque.z = wrench(5);

  wrenchDataPublisher_.publish(wdata);
}
