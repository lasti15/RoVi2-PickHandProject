#include <ros/ros.h>
#include <caros/robotiq_node.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robotiq2");

  ros::NodeHandle nh("~");
  std::string name = nh.getNamespace();
  caros::RobotiqNode hand(name, caros::RobotiqNode::ROBOTIQ2);
  hand.start();

  return 0;
}
