#include <caros/proxy_services_test_setup.h>

#include <caros/GripperSIProxy.hpp>
#include "gripper_service_interface_dummy.h"

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <list>
#include <functional>
#include <tuple>

using namespace caros::test;

// Container type to hold the services that should be tested
typedef std::list<std::tuple<std::function<bool(caros::GripperSIProxy&)>, const std::string>> services_t;

namespace
{
const services_t servicesToTest = {
  {std::make_tuple(
      std::bind(&caros::GripperSIProxy::moveQ, std::placeholders::_1, rw::math::Q()),
      "virtual bool GripperServiceInterfaceDummy::moveQ(const rw::math::Q&)")
  },
  {std::make_tuple(
      std::bind(&caros::GripperSIProxy::gripQ, std::placeholders::_1, rw::math::Q()),
      "virtual bool GripperServiceInterfaceDummy::gripQ(const rw::math::Q&)")
  },
  {std::make_tuple(
      std::bind(&caros::GripperSIProxy::setForceQ, std::placeholders::_1, rw::math::Q()),
      "virtual bool GripperServiceInterfaceDummy::setForceQ(const rw::math::Q&)")
  },
  {std::make_tuple(
      std::bind(&caros::GripperSIProxy::setVelocityQ, std::placeholders::_1, rw::math::Q()),
      "virtual bool GripperServiceInterfaceDummy::setVelocityQ(const rw::math::Q&)")
  },
  {std::make_tuple(
      std::bind(&caros::GripperSIProxy::stopMovement, std::placeholders::_1),
      "virtual bool GripperServiceInterfaceDummy::stopMovement()")
  }
};

typedef GripperServiceInterfaceDummy D_t;
typedef caros::GripperSIProxy P_t;
} // end namespace

TEST(GripperSIProxy, servicesSuccess)
{
  testServices<D_t, P_t, services_t>(servicesToTest, TestType::ReturnTrue);
}

TEST(GripperSIProxy, servicesFailure)
{
  testServices<D_t, P_t, services_t>(servicesToTest, TestType::ReturnFalse);
}

TEST(GripperSIProxy, unavailableService)
{
  testServices<D_t, P_t, services_t>(servicesToTest, TestType::UnavailableService);
}

TEST(GripperSIProxy, badServiceCall)
{
  testServices<D_t, P_t, services_t>(servicesToTest, TestType::BadServiceCall);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "gripper_service_interface_proxy");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
