#include <ros/ros.h>
#include <ros/console.h>
#include <caros/gripper_si_proxy.h>
#include <caros_common_msgs/caros_node_state.h>
#include <ros/topic.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "robotiq3_simple_demo");
  ros::NodeHandle n("~");

  const std::string nodeUnderTestName = "robotiq3_simple_demo_node";

  const std::string infoPrefix = ros::this_node::getName() + ": ";

  ROS_INFO_STREAM(ros::this_node::getName() << " started!");

  ROS_INFO_STREAM(infoPrefix << "Setting up GripperSIProxy");
  caros::GripperSIProxy r3Test(n, nodeUnderTestName);
  ROS_INFO_STREAM(infoPrefix << "GripperSIProxy setup. The hand should be initializing if it was not before!");

  const std::string nodeStateTopicName = nodeUnderTestName + "/" + "caros_node/caros_node_state";
  bool isRunning = false;
  while (!isRunning)
  {
    boost::shared_ptr<const caros_common_msgs::caros_node_state> nodeStateMessage;
    nodeStateMessage = ros::topic::waitForMessage<caros_common_msgs::caros_node_state>(nodeStateTopicName);
    isRunning = nodeStateMessage->state == "RUNNING";
  }

  ROS_INFO_STREAM(infoPrefix << "The hand should be initialized and ready to use now!");

  ros::spinOnce();

  ROS_INFO_STREAM(infoPrefix << "The hand is now at: " << r3Test.getQ());

  rw::math::Q target(4, 200, 200, 200, 100);
  ROS_INFO_STREAM(infoPrefix << "Trying to move to: " << target);

  bool ret = r3Test.moveQ(target);

  if (not ret)
  {
    ROS_ERROR_STREAM(infoPrefix << "Could not properly move the hand");
    return 1;
  }
  else
  {
    ROS_INFO_STREAM(infoPrefix << "Movement command successfully accepted");
  }

  rw::math::Q current = r3Test.getQ();

  while (ros::ok() && current != target)
  {
    ros::Duration(1).sleep();  // In seconds
    ros::spinOnce();
    current = r3Test.getQ();
    ROS_INFO_STREAM(infoPrefix << "The hand is now at: " << r3Test.getQ());
  }

  ROS_WARN_STREAM(infoPrefix << "This node will now end. This is intended behavior. When used with the test script this will lead to an error message. Please ignore that.");

  return 0;
}
