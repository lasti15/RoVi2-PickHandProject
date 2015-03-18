#include <caros/GripperSIProxy.hpp>
#include "gripper_service_interface_dummy.h"
#include <ros/ros.h>
#include <gtest/gtest.h>

#include <string>
#include <unordered_map>
#include <functional>
#include <tuple>

namespace
{
std::unordered_map<std::string, std::tuple<std::function<bool(caros::GripperSIProxy&)>, const std::string> > servicesToTest = {
  {"moveQ", std::make_tuple(
      std::bind(&caros::GripperSIProxy::moveQ, std::placeholders::_1, rw::math::Q()),
      "virtual bool GripperServiceInterfaceDummy::moveQ(const rw::math::Q&)")
  },
  {"gripQ", std::make_tuple(
      std::bind(&caros::GripperSIProxy::gripQ, std::placeholders::_1, rw::math::Q()),
      "virtual bool GripperServiceInterfaceDummy::gripQ(const rw::math::Q&)")
  },
  {"setForceQ", std::make_tuple(
      std::bind(&caros::GripperSIProxy::setForceQ, std::placeholders::_1, rw::math::Q()),
      "virtual bool GripperServiceInterfaceDummy::setForceQ(const rw::math::Q&)")
  },
  {"setVelocityQ", std::make_tuple(
      std::bind(&caros::GripperSIProxy::setVelocityQ, std::placeholders::_1, rw::math::Q()),
      "virtual bool GripperServiceInterfaceDummy::setVelocityQ(const rw::math::Q&)")
  },
  {"stopMovement", std::make_tuple(
      std::bind(&caros::GripperSIProxy::stopMovement, std::placeholders::_1),
      "virtual bool GripperServiceInterfaceDummy::stopMovement()")
  } };

void testReturnValue(std::function<bool()> doFunc, const std::string expectedFunctionCalled, const bool expectedReturnValue, const GripperServiceInterfaceDummy& si_dummy)
{
  if (expectedReturnValue == true)
  {
    EXPECT_TRUE(doFunc());
  }
  else
  {
    EXPECT_FALSE(doFunc());
  }
  EXPECT_EQ(expectedFunctionCalled, si_dummy.getMostRecentFunctionCalled());
}

void testReturnValueWrapper(const bool returnValueToTest)
{
  ros::NodeHandle nodehandleService("si_dummy");
  GripperServiceInterfaceDummy si_dummy(nodehandleService, returnValueToTest);

  ros::AsyncSpinner spinner(1);
  ASSERT_TRUE(spinner.canStart());
  spinner.start();

  ros::NodeHandle nodehandleClient("sip");
  caros::GripperSIProxy sip(nodehandleClient, "si_dummy");

  // The (unordered)map foreach uses member .first to access the key and .second to access the value
  for (const auto& serviceInfo : servicesToTest)
  {
    // Not using auto for all types, because it increases maintainability and makes it easier to diagnose compilation errors
    //auto serviceInfoFunc = std::get<0>(serviceInfo.second);
    std::function<bool(caros::GripperSIProxy&)> serviceInfoFunc = std::get<0>(serviceInfo.second);
    std::string serviceInfoFuncName = std::get<1>(serviceInfo.second);
    // Bind the SerialDeviceSIProxy member function to be called on the sip object
    std::function<bool()> func = std::bind(serviceInfoFunc, sip);

    testReturnValue(func, serviceInfoFuncName, returnValueToTest, si_dummy);
  }

  /* End */
  nodehandleClient.shutdown();
  nodehandleService.shutdown();

  spinner.stop();
}
} // end namespace

TEST(GripperSIProxy, servicesSuccess)
{
  testReturnValueWrapper(true);
}

TEST(GripperSIProxy, servicesFailure)
{
  testReturnValueWrapper(false);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "gripper_service_interface_proxy");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
