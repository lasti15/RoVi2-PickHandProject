#include <caros/test/universal_robot_test.h>

#include <ros/ros.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simple_caros_universalrobot_demo_using_move_ptp");

  const double q_change = 0.1;

  UrTest ur_test;
  bool ret = false;
  std::cout<<"Check 1"<<std::endl;
  ret = ur_test.testMovePtp(q_change);
   std::cout<<"Check 2"<<std::endl;
  if (!ret)
  {
    ROS_ERROR_STREAM("Could not properly do the testMovePtp");
    return 1;
  }

  return 0;
}
