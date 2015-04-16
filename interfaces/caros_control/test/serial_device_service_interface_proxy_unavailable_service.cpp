#include "serial_device_service_interface_proxy.h"

TEST(SerialDeviceSIProxy, unavailableService)
{
  caros::test::testServices<D_t, P_t, services_t>(servicesToTest, caros::test::TestType::UnavailableService);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "serial_device_service_interface_proxy");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
