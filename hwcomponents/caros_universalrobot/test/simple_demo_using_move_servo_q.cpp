#include "universal_robot_test.h"

#include <ros/ros.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ur_simple_demo");

  const double qChange = 0.2;

  UrTest urTest;
  bool ret = false;
  ret = urTest.testMoveServoQ(qChange);

  if (not ret)
  {
    ROS_ERROR_STREAM("Could not properly do the testMoveServoQ");
    return 1;
  }

  return 0;
}
