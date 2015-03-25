#include <caros/proxy_services_test_setup.h>

#include <caros/SerialDeviceSIProxy.hpp>
#include "serial_device_service_interface_dummy.h"

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <vector>
#include <functional>
#include <tuple>
#include <string>

namespace
{
typedef std::vector<std::tuple<std::function<bool(caros::SerialDeviceSIProxy&)>, const std::string>> services_t;

const services_t servicesToTest = {
  {std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::moveLin, std::placeholders::_1, rw::math::Transform3D<>(), 0, 0),
      "virtual bool SerialDeviceServiceInterfaceDummy::moveLin(const TransformAndSpeedContainer_t&)")
  },
  {std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::movePTP, std::placeholders::_1, rw::math::Q(), 0, 0),
      "virtual bool SerialDeviceServiceInterfaceDummy::movePTP(const QAndSpeedContainer_t&)")
  },
  {std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::movePTP_T, std::placeholders::_1, rw::math::Transform3D<>(), 0, 0),
      "virtual bool SerialDeviceServiceInterfaceDummy::movePTP_T(const TransformAndSpeedContainer_t&)")
  },
  {std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::moveServoQ, std::placeholders::_1, rw::math::Q(), 0),
      "virtual bool SerialDeviceServiceInterfaceDummy::moveServoQ(const QAndSpeedContainer_t&)")
  },
  {std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::moveServoT, std::placeholders::_1, rw::math::Transform3D<>(), 0),
      "virtual bool SerialDeviceServiceInterfaceDummy::moveServoT(const TransformAndSpeedContainer_t&)")
  },
  {std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::moveVelQ, std::placeholders::_1, rw::math::Q()),
      "virtual bool SerialDeviceServiceInterfaceDummy::moveVelQ(const rw::math::Q&)")
  },
  {std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::moveVelT, std::placeholders::_1, rw::math::VelocityScrew6D<>()),
      "virtual bool SerialDeviceServiceInterfaceDummy::moveVelT(const rw::math::VelocityScrew6D<double>&)")
  },
  {std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::moveLinFC, std::placeholders::_1, rw::math::Transform3D<>(), rw::math::Transform3D<>(), rw::math::Wrench6D<>(), rw::math::Q(6)),
      "virtual bool SerialDeviceServiceInterfaceDummy::moveLinFC(const rw::math::Transform3D<double>&, const rw::math::Transform3D<double>&, const rw::math::Wrench6D<double>&, const rw::math::Q&)")
  },
  {std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::stop, std::placeholders::_1),
      "virtual bool SerialDeviceServiceInterfaceDummy::moveStop()")
  },
  {std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::pause, std::placeholders::_1),
      "virtual bool SerialDeviceServiceInterfaceDummy::movePause()")
  },
  {std::make_tuple(
      std::bind(&caros::SerialDeviceSIProxy::setSafeModeEnabled, std::placeholders::_1, false),
      "virtual bool SerialDeviceServiceInterfaceDummy::moveSetSafeModeEnabled(bool)")
  }
};

typedef SerialDeviceServiceInterfaceDummy D_t;
typedef caros::SerialDeviceSIProxy P_t;
} // end namespace

TEST(SerialDeviceSIProxy, servicesSuccess)
{
  caros::test::testServices<D_t, P_t, services_t>(servicesToTest, caros::test::TestType::ReturnTrue);
}

TEST(SerialDeviceSIProxy, servicesFailure)
{
  caros::test::testServices<D_t, P_t, services_t>(servicesToTest, caros::test::TestType::ReturnFalse);
}

TEST(SerialDeviceSIProxy, unavailableService)
{
  caros::test::testServices<D_t, P_t, services_t>(servicesToTest, caros::test::TestType::UnavailableService);
}

TEST(SerialDeviceSIProxy, badServiceCall)
{
  caros::test::testServices<D_t, P_t, services_t>(servicesToTest, caros::test::TestType::BadServiceCall);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "serial_device_service_interface_proxy");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
