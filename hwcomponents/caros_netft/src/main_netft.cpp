#include <string>
#include <ros/ros.h>
#include <caros/netft_node.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "netft");
  ros::NodeHandle nh("~");
  std::string name = nh.getNamespace();
  caros::NetFTNode ftsensor(name);

  ftsensor.start();

  return 0;
}
