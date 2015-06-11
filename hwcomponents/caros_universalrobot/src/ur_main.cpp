#include <caros/universal_robots.h>

#include <caros/common_robwork.h>

#include <rw/loaders/WorkCellLoader.hpp>

#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "universalrobots");

  ros::NodeHandle nh("~");

  /* This workcell setup could also be moved into the UniversalRobots.cpp class - but leaving it as it is for now.
   */
  rw::models::WorkCell::Ptr workCell;
  workCell = caros::getWorkCell();
  if (workCell == NULL)
  {
    ROS_ERROR("No workcell was loaded - exiting...");
    return 1;
  }

  ROS_DEBUG("Workcell loaded - starting node.");

  caros::UniversalRobots ur(nh, workCell);
  ur.start();

  return 0;
}
