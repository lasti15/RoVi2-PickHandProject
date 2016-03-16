#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/String.h"
#include <iostream>
using namespace std;

int ImageNumber = 0;

void color_image_rawCallback(const sensor_msgs::ImageConstPtr& msg){
  try{
    cv::imshow ("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    imwrite( "/home/aitormig/workspace/RoVi/sample_images/Color_Image" << ImageNumber <<".jpg", cv_bridge::toCvShare(msg, "bgr8")->image );
    cout << "reading image" << endl;
    cv::waitKey();

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
void depth_image_rawCallback(const sensor_msgs::ImageConstPtr& msg){
  try{
    cv::imshow ("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    imwrite( "/home/aitormig/workspace/RoVi/sample_images/Depth_Image" << ImageNumber <<".jpg", cv_bridge::toCvShare(msg, "bgr8")->image );
    cout << "reading image" << endl;
    cv::waitKey();

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
void left_image_rawCallback(const sensor_msgs::ImageConstPtr& msg){
  try{
    cv::imshow ("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    imwrite( "/home/aitormig/workspace/RoVi/sample_images/Left_Image" << ImageNumber <<".jpg", cv_bridge::toCvShare(msg, "bgr8")->image );
    cout << "reading image" << endl;
    cv::waitKey();

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
void right_image_rawCallback(const sensor_msgs::ImageConstPtr& msg){
  try{
    cv::imshow ("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    imwrite( "/home/aitormig/workspace/RoVi/sample_images/Right_Image" << ImageNumber <<".jpg", cv_bridge::toCvShare(msg, "bgr8")->image );
    cout << "reading image" << endl;
    cv::waitKey();

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "image_saver");
  ros::NodeHandle n;

  cv::namedWindow("view");
  cv::startWindowThread();
  cout << "waiting" << endl;
  ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw", 1, color_image_rawCallback);
  ros::Subscriber sub2 =n.subscribe("/camera/rgb/image_raw", 1, depth_image_rawCallback);
  ros::Subscriber sub3 =n.subscribe("/camera/rgb/image_raw", 1, left_image_rawCallback);
  ros::Subscriber sub4 =n.subscribe("/camera/rgb/image_raw", 1, right_image_rawCallback);

  cv::destroyWindow("view");
  

  ros::spin();

  return 0;
}
