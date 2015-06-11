#include "ros/ros.h"
#include <caros/robotiq3_node.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robotiq3");

  ros::NodeHandle nh("~");
  std::string name = nh.getNamespace();
  caros::Robotiq3Node hand(name);
  hand.start();

  return 0;
}
