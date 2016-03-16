#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include <iostream>
#include <string>

int ImageNumber = 0;
std::stringstream Numss;

void color_image_rawCallback(const sensor_msgs::ImageConstPtr& msg){
  try{
    cv::imshow ("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        std::cout << "reading image" << std::endl;
    imwrite( "/home/aitormig/workspace/RoVi/sample_images/Color_Image" + Numss.str() + ".jpg", cv_bridge::toCvShare(msg, "bgr8")->image );
    cv::waitKey();

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
void depth_image_rawCallback(const sensor_msgs::ImageConstPtr& msg){
  std::cout << "reading image" << std::endl;
  try{
    //cv::imshow ("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    imwrite( "/home/aitormig/workspace/RoVi/sample_images/Depth_Image" + Numss.str() + ".jpg", cv_bridge::toCvShare(msg, "bgr8")->image );
    cv::waitKey(30);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
void left_image_rawCallback(const sensor_msgs::ImageConstPtr& msg){
  std::cout << "reading image" << std::endl;
  try{
    //cv::imshow ("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    imwrite( "/home/aitormig/workspace/RoVi/sample_images/Left_Image" + Numss.str() + ".jpg", cv_bridge::toCvShare(msg, "bgr8")->image );
    //cout << "reading image" << endl;
    cv::waitKey(30);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
void right_image_rawCallback(const sensor_msgs::ImageConstPtr& msg){
  std::cout << "reading image" << std::endl;
  try{
    //cv::imshow ("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    imwrite( "/home/aitormig/workspace/RoVi/sample_images/Right_Image" + Numss.str() + ".jpg", cv_bridge::toCvShare(msg, "bgr8")->image );
    //cout << "reading image" << endl;
    cv::waitKey(30);

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
  image_transport::ImageTransport it(n);


  cv::namedWindow("view");
  cv::startWindowThread();

  Numss.str(std::string());
  Numss << ImageNumber;
  ImageNumber++;

  std::cout << "waiting" << std::endl;
  
  image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, color_image_rawCallback);
  //image_transport::Subscriber sub2 =it.subscribe("camera/depth/image", 1, depth_image_rawCallback);
  //image_transport::Subscriber sub3 =it.subscribe("stereo_camera/left/image_raw", 1, left_image_rawCallback);
  //image_transport::Subscriber sub4 =it.subscribe("stereo_camera/right/image_raw", 1, right_image_rawCallback);
  
  std::cout << "waitingkey" << std::endl;  

  ros::spin();
  cv::destroyWindow("view");

  return 0;
}
