#include <caros/SerialDeviceSIProxy.hpp>
#include "serial_device_service_interface_dummy.h"
#include <ros/ros.h>
#include <gtest/gtest.h>

#include <string>
#include <unordered_map>
#include <functional>
#include <tuple>

namespace
{
std::unordered_map<std::string, std::tuple<std::function<bool(caros::SerialDeviceSIProxy&)>, const std::string> > servicesToTest = {
  {"moveLin", std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::moveLin, std::placeholders::_1, rw::math::Transform3D<>(), 0, 0),
      "virtual bool SerialDeviceServiceInterfaceDummy::moveLin(const TransformAndSpeedContainer_t&)")
  },
  {"movePTP", std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::movePTP, std::placeholders::_1, rw::math::Q(), 0, 0),
      "virtual bool SerialDeviceServiceInterfaceDummy::movePTP(const QAndSpeedContainer_t&)")
  },
  {"movePTP_T", std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::movePTP_T, std::placeholders::_1, rw::math::Transform3D<>(), 0, 0),
      "virtual bool SerialDeviceServiceInterfaceDummy::movePTP_T(const TransformAndSpeedContainer_t&)")
  },
  {"moveServoQ", std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::moveServoQ, std::placeholders::_1, rw::math::Q(), 0),
      "virtual bool SerialDeviceServiceInterfaceDummy::moveServoQ(const QAndSpeedContainer_t&)")
  },
  {"moveServoT", std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::moveServoT, std::placeholders::_1, rw::math::Transform3D<>(), 0),
      "virtual bool SerialDeviceServiceInterfaceDummy::moveServoT(const TransformAndSpeedContainer_t&)")
  },
  {"moveVelQ", std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::moveVelQ, std::placeholders::_1, rw::math::Q()),
      "virtual bool SerialDeviceServiceInterfaceDummy::moveVelQ(const rw::math::Q&)")
  },
  {"moveVelT", std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::moveVelT, std::placeholders::_1, rw::math::VelocityScrew6D<>()),
      "virtual bool SerialDeviceServiceInterfaceDummy::moveVelT(const rw::math::VelocityScrew6D<double>&)")
  },
  {"stop", std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::stop, std::placeholders::_1),
      "virtual bool SerialDeviceServiceInterfaceDummy::moveStop()")
  },
  {"pause", std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::pause, std::placeholders::_1),
      "virtual bool SerialDeviceServiceInterfaceDummy::movePause()")
  },
  {"setSafeModeEnabled", std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::setSafeModeEnabled, std::placeholders::_1, false),
      "virtual bool SerialDeviceServiceInterfaceDummy::moveSetSafeModeEnabled(bool)")
  } };

void testReturnValue(std::function<bool()> doFunc, const std::string expectedFunctionCalled, const bool expectedReturnValue, const SerialDeviceServiceInterfaceDummy& sdsid)
{
  if (expectedReturnValue == true)
  {
    EXPECT_TRUE(doFunc());
  }
  else
  {
    EXPECT_FALSE(doFunc());
  }
  EXPECT_EQ(expectedFunctionCalled, sdsid.getMostRecentFunctionCalled());
}

void testReturnValueWrapper(const bool returnValueToTest)
{
  ros::NodeHandle nodehandleService("sdsi_dummy");
  SerialDeviceServiceInterfaceDummy sdsid(nodehandleService, returnValueToTest);

  ros::AsyncSpinner spinner(1);
  ASSERT_TRUE(spinner.canStart());
  spinner.start();

  ros::NodeHandle nodehandleClient("sip");
  caros::SerialDeviceSIProxy sip(nodehandleClient, "sdsi_dummy");

  // The (unordered)map foreach uses member .first to access the key and .second to access the value
  for (const auto& serviceInfo : servicesToTest)
  {
    // Not using auto for all types, because it increases maintainability and makes it easier to diagnose compilation errors
    auto serviceInfoFunc = std::get<0>(serviceInfo.second);
    std::string serviceInfoFuncName = std::get<1>(serviceInfo.second);
    // Bind the SerialDeviceSIProxy member function to be called on the sip object
    std::function<bool()> func = std::bind(serviceInfoFunc, sip);

    testReturnValue(func, serviceInfoFuncName, returnValueToTest, sdsid);
  }

  /* End */
  nodehandleClient.shutdown();
  nodehandleService.shutdown();

  spinner.stop();
}
} // end namespace

TEST(SerialDeviceSIProxy, servicesSuccess)
{
  testReturnValueWrapper(true);
}

TEST(SerialDeviceSIProxy, servicesFailure)
{
  testReturnValueWrapper(false);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "serial_device_service_interface_proxy");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
