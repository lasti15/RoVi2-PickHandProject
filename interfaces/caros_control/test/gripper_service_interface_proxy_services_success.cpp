#include "gripper_service_interface_proxy.h"

TEST(GripperSIProxy, servicesSuccess)
{
  caros::test::testServices<D_t, P_t, services_t>(servicesToTest, caros::test::TestType::ReturnTrue);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "gripper_service_interface_proxy");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
