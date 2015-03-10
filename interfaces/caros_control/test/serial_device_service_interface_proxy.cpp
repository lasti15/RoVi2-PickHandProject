#include <caros/SerialDeviceSIProxy.hpp>
#include "serial_device_service_interface_dummy.h"
#include <ros/ros.h>
#include <gtest/gtest.h>

#include <string>
#include <unordered_map>

namespace
{
std::unordered_map<std::string, const std::string> functionCalledMap = {
  {"moveLin", "virtual bool SerialDeviceServiceInterfaceDummy::moveLin(const TransformAndSpeedContainer_t&)"},
  {"movePTP", "virtual bool SerialDeviceServiceInterfaceDummy::movePTP(const QAndSpeedContainer_t&)"},
  {"movePTP_T", "virtual bool SerialDeviceServiceInterfaceDummy::movePTP_T(const TransformAndSpeedContainer_t&)"},
  {"moveServoQ", "virtual bool SerialDeviceServiceInterfaceDummy::moveServoQ(const QAndSpeedContainer_t&)"},
  {"moveServoT", "virtual bool SerialDeviceServiceInterfaceDummy::moveServoT(const TransformAndSpeedContainer_t&)"},
  {"moveVelQ", "virtual bool SerialDeviceServiceInterfaceDummy::moveVelQ(const rw::math::Q&)"},
  {"moveVelT", "virtual bool SerialDeviceServiceInterfaceDummy::moveVelT(const rw::math::VelocityScrew6D<double>&)"},
  {"stop", "virtual bool SerialDeviceServiceInterfaceDummy::moveStop()"},
  {"pause", "virtual bool SerialDeviceServiceInterfaceDummy::movePause()"},
  {"setSafeModeEnabled", "virtual bool SerialDeviceServiceInterfaceDummy::moveSetSafeModeEnabled(bool)"} };
}


TEST(SerialDeviceSIProxy, servicesSuccess)
{
  ros::NodeHandle nodehandleService("sdsi_dummy_true");
  SerialDeviceServiceInterfaceDummy sdsid(nodehandleService, true);

  ros::AsyncSpinner spinner(1);
  ASSERT_TRUE(spinner.canStart());
  spinner.start();

  ros::NodeHandle nodehandleClient("sip");
  caros::SerialDeviceSIProxy sip(nodehandleClient, "sdsi_dummy_true");

  EXPECT_TRUE(sip.moveLin(rw::math::Transform3D<>(), 0, 0));
  EXPECT_EQ(functionCalledMap.at("moveLin"), sdsid.getMostRecentFunctionCalled());

  EXPECT_TRUE(sip.movePTP(rw::math::Q(), 0, 0));
  EXPECT_EQ(functionCalledMap.at("movePTP"), sdsid.getMostRecentFunctionCalled());

  EXPECT_TRUE(sip.movePTP_T(rw::math::Transform3D<>(), 0, 0));
  EXPECT_EQ(functionCalledMap.at("movePTP_T"), sdsid.getMostRecentFunctionCalled());

  EXPECT_TRUE(sip.moveServoQ(rw::math::Q(), 0));
  EXPECT_EQ(functionCalledMap.at("moveServoQ"), sdsid.getMostRecentFunctionCalled());

  EXPECT_TRUE(sip.moveServoT(rw::math::Transform3D<>(), 0));
  EXPECT_EQ(functionCalledMap.at("moveServoT"), sdsid.getMostRecentFunctionCalled());

  EXPECT_TRUE(sip.moveVelQ(rw::math::Q()));
  EXPECT_EQ(functionCalledMap.at("moveVelQ"), sdsid.getMostRecentFunctionCalled());

  EXPECT_TRUE(sip.moveVelT(rw::math::VelocityScrew6D<>()));
  EXPECT_EQ(functionCalledMap.at("moveVelT"), sdsid.getMostRecentFunctionCalled());

  /*
    float selection[6];
    EXPECT_TRUE(sip.moveLinFC(rw::math::Transform3D<>(), rw::math::Wrench6D<>(), selection, "",
    rw::math::Transform3D<>(), 0, 0));
    EXPECT_EQ(functionCalledMap.at("moveLinFC"), sdsid.getMostRecentFunctionCalled());
  */

  EXPECT_TRUE(sip.stop());
  EXPECT_EQ(functionCalledMap.at("stop"), sdsid.getMostRecentFunctionCalled());

  EXPECT_TRUE(sip.pause());
  EXPECT_EQ(functionCalledMap.at("pause"), sdsid.getMostRecentFunctionCalled());

  EXPECT_TRUE(sip.setSafeModeEnabled(false));
  EXPECT_EQ(functionCalledMap.at("setSafeModeEnabled"), sdsid.getMostRecentFunctionCalled());

  /* End */
  nodehandleClient.shutdown();
  nodehandleService.shutdown();

  spinner.stop();
}

/* An exact copy of servicesSuccess, just with true->false substitution (although a few places shouldn't have the
 * substitution done, so verify each location in the find-and-replace process */
TEST(SerialDeviceSIProxy, servicesFailure)
{
  ros::NodeHandle nodehandleService("sdsi_dummy_false");
  SerialDeviceServiceInterfaceDummy sdsid(nodehandleService, false);

  ros::AsyncSpinner spinner(1);
  ASSERT_TRUE(spinner.canStart());
  spinner.start();

  ros::NodeHandle nodehandleClient("sip");
  caros::SerialDeviceSIProxy sip(nodehandleClient, "sdsi_dummy_false");

  EXPECT_FALSE(sip.moveLin(rw::math::Transform3D<>(), 0, 0));
  EXPECT_EQ(functionCalledMap.at("moveLin"), sdsid.getMostRecentFunctionCalled());

  EXPECT_FALSE(sip.movePTP(rw::math::Q(), 0, 0));
  EXPECT_EQ(functionCalledMap.at("movePTP"), sdsid.getMostRecentFunctionCalled());

  EXPECT_FALSE(sip.movePTP_T(rw::math::Transform3D<>(), 0, 0));
  EXPECT_EQ(functionCalledMap.at("movePTP_T"), sdsid.getMostRecentFunctionCalled());

  EXPECT_FALSE(sip.moveServoQ(rw::math::Q(), 0));
  EXPECT_EQ(functionCalledMap.at("moveServoQ"), sdsid.getMostRecentFunctionCalled());

  EXPECT_FALSE(sip.moveServoT(rw::math::Transform3D<>(), 0));
  EXPECT_EQ(functionCalledMap.at("moveServoT"), sdsid.getMostRecentFunctionCalled());

  EXPECT_FALSE(sip.moveVelQ(rw::math::Q()));
  EXPECT_EQ(functionCalledMap.at("moveVelQ"), sdsid.getMostRecentFunctionCalled());

  EXPECT_FALSE(sip.moveVelT(rw::math::VelocityScrew6D<>()));
  EXPECT_EQ(functionCalledMap.at("moveVelT"), sdsid.getMostRecentFunctionCalled());

  /*
    float selection[6];
    EXPECT_FALSE(sip.moveLinFC(rw::math::Transform3D<>(), rw::math::Wrench6D<>(), selection, "",
    rw::math::Transform3D<>(), 0, 0));
    EXPECT_EQ(functionCalledMap.at("moveLinFC"), sdsid.getMostRecentFunctionCalled());
  */

  EXPECT_FALSE(sip.stop());
  EXPECT_EQ(functionCalledMap.at("stop"), sdsid.getMostRecentFunctionCalled());

  EXPECT_FALSE(sip.pause());
  EXPECT_EQ(functionCalledMap.at("pause"), sdsid.getMostRecentFunctionCalled());

  EXPECT_FALSE(sip.setSafeModeEnabled(false));
  EXPECT_EQ(functionCalledMap.at("setSafeModeEnabled"), sdsid.getMostRecentFunctionCalled());

  /* End */
  nodehandleClient.shutdown();
  nodehandleService.shutdown();

  spinner.stop();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "serial_device_service_interface_proxy");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
