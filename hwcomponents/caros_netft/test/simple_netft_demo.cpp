#include <ros/ros.h>
#include <ros/console.h>
#include <caros/ft_sensor_si_proxy.h>
#include <caros_common_msgs/CarosNodeState.h>
#include <ros/topic.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "netft_simple_demo");
  ros::NodeHandle n("~");
  const std::string info_prefix = ros::this_node::getName() + ": ";
  const std::string node_under_test_name = "netft_simple_demo_node";

  ROS_INFO_STREAM(info_prefix << "Setting up FTSensorSIProxy");
  caros::FTSensorSIProxy ft_si_proxy(n, node_under_test_name);
  ROS_INFO_STREAM(info_prefix << "FTSensorSIProxy setup. The sensor has been initialized!");

  const std::string node_stateTopicName = node_under_test_name + "/" + "caros_node/caros_node_state";
  bool is_running = false;
  while (not is_running)
  {
    auto node_state_message = ros::topic::waitForMessage<caros_common_msgs::CarosNodeState>(node_stateTopicName);
    is_running = node_state_message->state == "RUNNING";
  }

  while (ros::ok())
  {
    rw::math::Wrench6D<> wrench = ft_si_proxy.getWrench();
    ROS_INFO_STREAM(info_prefix << "The hand is now at: " << wrench);    
    ros::Duration(1).sleep();  // In seconds
    ros::spinOnce();
  }

  ROS_WARN_STREAM(info_prefix << "This node will now end. This is intended behavior. When used with the test script "
                                 "this will lead to an error message. Please ignore that.");

  return 0;
}
