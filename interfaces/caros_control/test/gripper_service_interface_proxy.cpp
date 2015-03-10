#include <caros/GripperSIProxy.hpp>
#include "gripper_service_interface_dummy.h"
#include <ros/ros.h>
#include <gtest/gtest.h>

#include <string>
#include <unordered_map>

namespace
{
std::unordered_map<std::string, const std::string> functionCalledMap = {
  {"moveQ", "virtual bool GripperServiceInterfaceDummy::moveQ(const rw::math::Q&)"},
  {"gripQ", "virtual bool GripperServiceInterfaceDummy::gripQ(const rw::math::Q&)"},
  {"setForceQ", "virtual bool GripperServiceInterfaceDummy::setForceQ(const rw::math::Q&)"},
  {"setVelocityQ", "virtual bool GripperServiceInterfaceDummy::setVelocityQ(const rw::math::Q&)"},
  {"stopMovement", "virtual bool GripperServiceInterfaceDummy::stopMovement()"} };
}

TEST(GripperSIProxy, servicesSuccess)
{
  ros::NodeHandle nodehandleService("gsi_dummy_true");
  GripperServiceInterfaceDummy gsid(nodehandleService, true);

  ros::AsyncSpinner spinner(1);
  ASSERT_TRUE(spinner.canStart());
  spinner.start();

  ros::NodeHandle nodehandleClient("sip");
  caros::GripperSIProxy sip(nodehandleClient, "gsi_dummy_true");

  EXPECT_TRUE(sip.moveQ(rw::math::Q()));
  EXPECT_EQ(functionCalledMap.at("moveQ"), gsid.getMostRecentFunctionCalled());

  EXPECT_TRUE(sip.gripQ(rw::math::Q()));
  EXPECT_EQ(functionCalledMap.at("gripQ"), gsid.getMostRecentFunctionCalled());

  EXPECT_TRUE(sip.setForceQ(rw::math::Q()));
  EXPECT_EQ(functionCalledMap.at("setForceQ"), gsid.getMostRecentFunctionCalled());

  EXPECT_TRUE(sip.setVelocityQ(rw::math::Q()));
  EXPECT_EQ(functionCalledMap.at("setVelocityQ"), gsid.getMostRecentFunctionCalled());

  EXPECT_TRUE(sip.stopMovement());
  EXPECT_EQ(functionCalledMap.at("stopMovement"), gsid.getMostRecentFunctionCalled());

  /* End */
  nodehandleClient.shutdown();
  nodehandleService.shutdown();

  spinner.stop();
}

TEST(GripperSIProxy, servicesFailure)
{
  ros::NodeHandle nodehandleService("gsi_dummy_true");
  GripperServiceInterfaceDummy gsid(nodehandleService, false);

  ros::AsyncSpinner spinner(1);
  ASSERT_TRUE(spinner.canStart());
  spinner.start();

  ros::NodeHandle nodehandleClient("sip");
  caros::GripperSIProxy sip(nodehandleClient, "gsi_dummy_true");

  EXPECT_FALSE(sip.moveQ(rw::math::Q()));
  EXPECT_EQ(functionCalledMap.at("moveQ"), gsid.getMostRecentFunctionCalled());

  EXPECT_FALSE(sip.gripQ(rw::math::Q()));
  EXPECT_EQ(functionCalledMap.at("gripQ"), gsid.getMostRecentFunctionCalled());

  EXPECT_FALSE(sip.setForceQ(rw::math::Q()));
  EXPECT_EQ(functionCalledMap.at("setForceQ"), gsid.getMostRecentFunctionCalled());

  EXPECT_FALSE(sip.setVelocityQ(rw::math::Q()));
  EXPECT_EQ(functionCalledMap.at("setVelocityQ"), gsid.getMostRecentFunctionCalled());

  EXPECT_FALSE(sip.stopMovement());
  EXPECT_EQ(functionCalledMap.at("stopMovement"), gsid.getMostRecentFunctionCalled());

  /* End */
  nodehandleClient.shutdown();
  nodehandleService.shutdown();

  spinner.stop();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "gripper_service_interface_proxy");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
