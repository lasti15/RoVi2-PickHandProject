#include <caros/universal_robots.h>

#include <caros/common_robwork.h>

#include <rw/loaders/WorkCellLoader.hpp>

#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "universalrobots");

  ros::NodeHandle nh("~");

  caros::UniversalRobots ur(nh);
  ur.start();

  return 0;
}
