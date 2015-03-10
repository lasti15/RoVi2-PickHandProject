#include <caros/common.h>
#include <gtest/gtest.h>

#include <rw/loaders/WorkCellFactory.hpp>

namespace
{
void testQAbsoluteDifference(const rw::math::Q& value1, const rw::math::Q& value2, const rw::math::Q& difference)
{
  // Make sure all the Q's have the same size
  ASSERT_EQ(value1.size(), value2.size());
  ASSERT_EQ(value1.size(), difference.size());

  for (std::size_t index = 0; index < value1.size(); ++index)
  {
    EXPECT_DOUBLE_EQ(std::fabs(value1[index] - value2[index]), difference[index]);
  }
}

void testQEqual(const rw::math::Q& value1, const rw::math::Q& value2)
{
  // Make sure all the Q's have the same size
  ASSERT_EQ(value1.size(), value2.size());

  for (std::size_t index = 0; index < value1.size(); ++index)
  {
    EXPECT_DOUBLE_EQ(value1[index], value2[index]);
  }
}
}

TEST(TypeConversion, state)
{
  rw::models::WorkCell::Ptr wc;
  ASSERT_TRUE(wc == 0);
  EXPECT_NO_THROW({wc = rw::loaders::WorkCellFactory::load("data/workcell.xml");});
  ASSERT_TRUE(wc != 0);
  const auto device = wc->findDevice("UR1");
  ASSERT_TRUE(device != 0);

  const auto defaultState = wc->getDefaultState();
  auto newState = defaultState;

  rw::math::Q qChange(device->getQ(newState).size(), 1.3);
  for (std::size_t index = 0; index < qChange.size(); ++index)
  {
    qChange[index] += index;
  }
  device->setQ(device->getQ(newState) + qChange, newState);

  // Verify that the states give different Q's
  testQAbsoluteDifference(device->getQ(newState), device->getQ(defaultState), qChange);

  const auto toRos = caros::toRos(newState);
  // Test the function taking a workcell ptr
  const auto stateFromRos = caros::toRw(toRos, wc);
  testQEqual(device->getQ(newState), device->getQ(stateFromRos));
  testQAbsoluteDifference(device->getQ(stateFromRos), device->getQ(defaultState), qChange);

  // Test the function taking a RobWork state reference
  auto modifiedState = wc->getDefaultState();
  // Verify that the modified state is similar to defaultState
  testQEqual(device->getQ(defaultState), device->getQ(modifiedState));
  caros::toRw(toRos, modifiedState);
  testQEqual(device->getQ(newState), device->getQ(modifiedState));
  testQAbsoluteDifference(device->getQ(defaultState), device->getQ(modifiedState), qChange);
}

/************************************************************************
 * Q
 ************************************************************************/
TEST(TypeConversion, toRosFromRwQ)
{
  const rw::math::Q rwQ(6, 3.0);
  const auto toRosQ = caros::toRos(rwQ);
  ASSERT_EQ(toRosQ.data.size(), rwQ.size());
  for (std::size_t index = 0; index < rwQ.size(); ++index)
  {
    EXPECT_DOUBLE_EQ(toRosQ.data.at(index), rwQ[index]);
  }

  /* Forth and back */
  const auto toRosAndBack = caros::toRw(caros::toRos(rwQ));
  // Note: Currently the comparison operator implementation is not doing proper floating point comparisons
  EXPECT_TRUE(rwQ == toRosAndBack);
  testQEqual(rwQ, toRosAndBack);
}

TEST(TypeConversion, toRwFromRosQ)
{
  // Create ros Q and fill it with data
  caros_common_msgs::Q rosQ;
  rosQ.data.resize(4);
  for (std::size_t index = 0; index < rosQ.data.size(); ++index)
  {
    rosQ.data.at(index) = static_cast<double>(1.0 + index);
  }

  // Convert to robwork Q
  const auto toRwQ = caros::toRw(rosQ);
  ASSERT_EQ(toRwQ.size(), rosQ.data.size());
  for (std::size_t index = 0; index < rosQ.data.size(); ++index)
  {
    EXPECT_DOUBLE_EQ(toRwQ[index], rosQ.data.at(index));
  }

  /* Forth and back */
  const auto toRwAndBack = caros::toRos(caros::toRw(rosQ));
  ASSERT_EQ(rosQ.data.size(), toRwAndBack.data.size());
  for (std::size_t index = 0; index < rosQ.data.size(); ++index)
  {
    EXPECT_DOUBLE_EQ(rosQ.data.at(index), toRwAndBack.data.at(index));
  }
}

/************************************************************************
 * Transform
 ************************************************************************/
TEST(TypeConversion, toRosFromRwTransform3D)
{
  // The comparisonPrecision value is found by trial and error.
  const double comparisonPrecision = 1.0e-05;

  /* 5 randomly generated Transform3D - could also do this with seeded Math::ranTransform3D calls */
  const rw::math::Transform3D<> transforms[] = {
    {rw::math::Vector3D<>(0.821768, -0.294349, -0.48791), rw::math::Rotation3D<>(-0.182442, -0.931498, 0.314686, -0.810775, -0.0385244, -0.584088, 0.5562, -0.361702, -0.748207)},
    {rw::math::Vector3D<>(0.0947497, -0.968762, 0.229179), rw::math::Rotation3D<>(-0.149109, -0.717196, 0.680732, -0.765136, 0.519767, 0.380011, -0.626365, -0.46419, -0.626255)},
    {rw::math::Vector3D<>(-0.234126, -0.938533, 0.253656), rw::math::Rotation3D<>(-0.279391, 0.75009, 0.599421, 0.238234, -0.550607, 0.800048, 0.930153, 0.366328, -0.0248621)},
    {rw::math::Vector3D<>(-0.252482, -0.967311, 0.0237235), rw::math::Rotation3D<>(0.439868, 0.168623, 0.88209, 0.595296, -0.790167, -0.145804, 0.672412, 0.589239, -0.44795)},
    {rw::math::Vector3D<>(-0.779311, 0.626539, 0.0110964), rw::math::Rotation3D<>(-0.0360424, -0.433045, -0.900652, 0.948205, -0.299441, 0.10603, -0.315608, -0.850181, 0.421408)}};

  for (const auto& transform : transforms)
  {
    const auto toRos = caros::toRos(transform);
    const rw::math::Vector3D<> translation(toRos.translation.x, toRos.translation.y, toRos.translation.z);
    const rw::math::Quaternion<> quaternion(toRos.rotation.x, toRos.rotation.y, toRos.rotation.z, toRos.rotation.w);
    const rw::math::Transform3D<> reconstructedTransform(translation, quaternion.toRotation3D());

    EXPECT_TRUE(transform.equal(reconstructedTransform, comparisonPrecision));

    /* Forth and back */
    const auto toRosAndBack = caros::toRw(caros::toRos(transform));
    EXPECT_TRUE(transform.equal(toRosAndBack, comparisonPrecision));
  }

}

TEST(TypeConversion, toRwFromRosTransform)
{
  // The comparisonPrecision value is found by trial and error.
  const double comparisonPrecision = 1.0e-04;

  /* 5 randomly generated Transform3D - could also do this with seeded Math::ranTransform3D calls */
  const rw::math::Transform3D<> rwTransforms[] = {
    {rw::math::Vector3D<>(0.821768, -0.294349, -0.48791), rw::math::Rotation3D<>(-0.182442, -0.931498, 0.314686, -0.810775, -0.0385244, -0.584088, 0.5562, -0.361702, -0.748207)},
    {rw::math::Vector3D<>(0.0947497, -0.968762, 0.229179), rw::math::Rotation3D<>(-0.149109, -0.717196, 0.680732, -0.765136, 0.519767, 0.380011, -0.626365, -0.46419, -0.626255)},
    {rw::math::Vector3D<>(-0.234126, -0.938533, 0.253656), rw::math::Rotation3D<>(-0.279391, 0.75009, 0.599421, 0.238234, -0.550607, 0.800048, 0.930153, 0.366328, -0.0248621)},
    {rw::math::Vector3D<>(-0.252482, -0.967311, 0.0237235), rw::math::Rotation3D<>(0.439868, 0.168623, 0.88209, 0.595296, -0.790167, -0.145804, 0.672412, 0.589239, -0.44795)},
    {rw::math::Vector3D<>(-0.779311, 0.626539, 0.0110964), rw::math::Rotation3D<>(-0.0360424, -0.433045, -0.900652, 0.948205, -0.299441, 0.10603, -0.315608, -0.850181, 0.421408)}};

  for (const auto& rwTransform : rwTransforms)
  {
    geometry_msgs::Transform transform;
    transform.translation.x = rwTransform.P()[0];
    transform.translation.y = rwTransform.P()[1];
    transform.translation.z = rwTransform.P()[2];
    const rw::math::Quaternion<> rwQuaternion(rwTransform.R());
    transform.rotation.x = rwQuaternion.getQx();
    transform.rotation.y = rwQuaternion.getQy();
    transform.rotation.z = rwQuaternion.getQz();
    transform.rotation.w = rwQuaternion.getQw();

    const auto toRw = caros::toRw(transform);
    EXPECT_TRUE(rwTransform.equal(toRw, comparisonPrecision));

    /* Forth and back */
    const auto toRwAndBack = caros::toRos(caros::toRw(transform));
    EXPECT_NEAR(transform.translation.x, toRwAndBack.translation.x, comparisonPrecision);
    EXPECT_NEAR(transform.translation.y, toRwAndBack.translation.y, comparisonPrecision);
    EXPECT_NEAR(transform.translation.z, toRwAndBack.translation.z, comparisonPrecision);
    EXPECT_NEAR(transform.rotation.x, toRwAndBack.rotation.x, comparisonPrecision);
    EXPECT_NEAR(transform.rotation.y, toRwAndBack.rotation.y, comparisonPrecision);
    EXPECT_NEAR(transform.rotation.z, toRwAndBack.rotation.z, comparisonPrecision);
    EXPECT_NEAR(transform.rotation.w, toRwAndBack.rotation.w, comparisonPrecision);
  }
}

/************************************************************************
 * Pose
 ************************************************************************/
TEST(TypeConversion, toRosPoseFromRwTransform3D)
{
  /********************************
   * Notes:
   * Basically copy/paste from toRos_from_rw_transform3D with the following replacements
   * caros::toRos -> caros::toRosPose
   * toRos.translation -> toRos.position
   * toRos.rotation -> toRos.orientation
   ********************************/
  // The comparisonPrecision value is found by trial and error.
  const double comparisonPrecision = 1.0e-05;

    /* 5 randomly generated Transform3D - could also do this with seeded Math::ranTransform3D calls */
  const rw::math::Transform3D<> transforms[] = {
    {rw::math::Vector3D<>(0.821768, -0.294349, -0.48791), rw::math::Rotation3D<>(-0.182442, -0.931498, 0.314686, -0.810775, -0.0385244, -0.584088, 0.5562, -0.361702, -0.748207)},
    {rw::math::Vector3D<>(0.0947497, -0.968762, 0.229179), rw::math::Rotation3D<>(-0.149109, -0.717196, 0.680732, -0.765136, 0.519767, 0.380011, -0.626365, -0.46419, -0.626255)},
    {rw::math::Vector3D<>(-0.234126, -0.938533, 0.253656), rw::math::Rotation3D<>(-0.279391, 0.75009, 0.599421, 0.238234, -0.550607, 0.800048, 0.930153, 0.366328, -0.0248621)},
    {rw::math::Vector3D<>(-0.252482, -0.967311, 0.0237235), rw::math::Rotation3D<>(0.439868, 0.168623, 0.88209, 0.595296, -0.790167, -0.145804, 0.672412, 0.589239, -0.44795)},
    {rw::math::Vector3D<>(-0.779311, 0.626539, 0.0110964), rw::math::Rotation3D<>(-0.0360424, -0.433045, -0.900652, 0.948205, -0.299441, 0.10603, -0.315608, -0.850181, 0.421408)}};

  for (const auto& transform : transforms)
  {
    const auto toRos = caros::toRosPose(transform);
    const rw::math::Vector3D<> position(toRos.position.x, toRos.position.y, toRos.position.z);
    const rw::math::Quaternion<> quaternion(toRos.orientation.x, toRos.orientation.y, toRos.orientation.z, toRos.orientation.w);
    const rw::math::Transform3D<> reconstructedTransform(position, quaternion.toRotation3D());

    EXPECT_TRUE(transform.equal(reconstructedTransform, comparisonPrecision));

    /* Forth and back */
    const auto toRosPoseAndBack = caros::toRw(caros::toRosPose(transform));
    EXPECT_TRUE(transform.equal(toRosPoseAndBack, comparisonPrecision));
  }
}

TEST(TypeConversion, toRwFromRosPose)
{
  /********************************
   * Notes:
   * Basically copy/paste from toRw_from_ros_transform with a bit more extensive changes.
   ********************************/
  // The comparisonPrecision value is found by trial and error.
  const double comparisonPrecision = 1.0e-04;

  /* 5 randomly generated Transform3D - could also do this with seeded Math::ranTransform3D calls */
  const rw::math::Transform3D<> rwTransforms[] = {
    {rw::math::Vector3D<>(0.821768, -0.294349, -0.48791), rw::math::Rotation3D<>(-0.182442, -0.931498, 0.314686, -0.810775, -0.0385244, -0.584088, 0.5562, -0.361702, -0.748207)},
    {rw::math::Vector3D<>(0.0947497, -0.968762, 0.229179), rw::math::Rotation3D<>(-0.149109, -0.717196, 0.680732, -0.765136, 0.519767, 0.380011, -0.626365, -0.46419, -0.626255)},
    {rw::math::Vector3D<>(-0.234126, -0.938533, 0.253656), rw::math::Rotation3D<>(-0.279391, 0.75009, 0.599421, 0.238234, -0.550607, 0.800048, 0.930153, 0.366328, -0.0248621)},
    {rw::math::Vector3D<>(-0.252482, -0.967311, 0.0237235), rw::math::Rotation3D<>(0.439868, 0.168623, 0.88209, 0.595296, -0.790167, -0.145804, 0.672412, 0.589239, -0.44795)},
    {rw::math::Vector3D<>(-0.779311, 0.626539, 0.0110964), rw::math::Rotation3D<>(-0.0360424, -0.433045, -0.900652, 0.948205, -0.299441, 0.10603, -0.315608, -0.850181, 0.421408)}};

  for (const auto& rwTransform : rwTransforms)
  {
    geometry_msgs::Pose pose;
    pose.position.x = rwTransform.P()[0];
    pose.position.y = rwTransform.P()[1];
    pose.position.z = rwTransform.P()[2];
    const rw::math::Quaternion<> rwQuaternion(rwTransform.R());
    pose.orientation.x = rwQuaternion.getQx();
    pose.orientation.y = rwQuaternion.getQy();
    pose.orientation.z = rwQuaternion.getQz();
    pose.orientation.w = rwQuaternion.getQw();

    const auto toRw = caros::toRw(pose);
    EXPECT_TRUE(rwTransform.equal(toRw, comparisonPrecision));

    /* Forth and back */
    const auto toRwAndBack = caros::toRosPose(caros::toRw(pose));
    EXPECT_NEAR(pose.position.x, toRwAndBack.position.x, comparisonPrecision);
    EXPECT_NEAR(pose.position.y, toRwAndBack.position.y, comparisonPrecision);
    EXPECT_NEAR(pose.position.z, toRwAndBack.position.z, comparisonPrecision);
    EXPECT_NEAR(pose.orientation.x, toRwAndBack.orientation.x, comparisonPrecision);
    EXPECT_NEAR(pose.orientation.y, toRwAndBack.orientation.y, comparisonPrecision);
    EXPECT_NEAR(pose.orientation.z, toRwAndBack.orientation.z, comparisonPrecision);
    EXPECT_NEAR(pose.orientation.w, toRwAndBack.orientation.w, comparisonPrecision);
  }
}

/************************************************************************
 * Wrench
 ************************************************************************/
TEST(TypeConversion, toRosFromRwWrench6D)
{
  const rw::math::Wrench6D<> wrenches[] = {
    {0.135477004107, 0.905791934114, 0.835008589784, 0.126986811869, 0.968867771095, 0.913375855656},
    {0.22103404277, 0.632359249983, 0.30816705036, 0.0975404016208, 0.547220596345, 0.278498218395},
    {0.188381975982, 0.546881519025, 0.99288130179, 0.95750682964, 0.996461325325, 0.96488853381},
    {0.967694936786, 0.157613076502, 0.725838963175, 0.970592778875, 0.981109691551, 0.957166949753},
    {0.109861750621, 0.485375648364, 0.798105856637, 0.800280473195, 0.297029449372, 0.141886345111}};

  for (const auto& wrench : wrenches)
  {
    const auto toRos = caros::toRos(wrench);
    EXPECT_DOUBLE_EQ(wrench(0), toRos.force.x);
    EXPECT_DOUBLE_EQ(wrench(1), toRos.force.y);
    EXPECT_DOUBLE_EQ(wrench(2), toRos.force.z);
    EXPECT_DOUBLE_EQ(wrench(3), toRos.torque.x);
    EXPECT_DOUBLE_EQ(wrench(4), toRos.torque.y);
    EXPECT_DOUBLE_EQ(wrench(5), toRos.torque.z);

    /* Forth and back */
    const auto toRosAndBack = caros::toRw(caros::toRos(wrench));
    EXPECT_DOUBLE_EQ(wrench(0), toRosAndBack(0));
    EXPECT_DOUBLE_EQ(wrench(1), toRosAndBack(1));
    EXPECT_DOUBLE_EQ(wrench(2), toRosAndBack(2));
    EXPECT_DOUBLE_EQ(wrench(3), toRosAndBack(3));
    EXPECT_DOUBLE_EQ(wrench(4), toRosAndBack(4));
    EXPECT_DOUBLE_EQ(wrench(5), toRosAndBack(5));
  }
}

TEST(TypeConversion, toRwFromRosWrench)
{
  const double wrenchValues[][6] = {
    {0.135477004107, 0.905791934114, 0.835008589784, 0.126986811869, 0.968867771095, 0.913375855656},
    {0.22103404277, 0.632359249983, 0.30816705036, 0.0975404016208, 0.547220596345, 0.278498218395},
    {0.188381975982, 0.546881519025, 0.99288130179, 0.95750682964, 0.996461325325, 0.96488853381},
    {0.967694936786, 0.157613076502, 0.725838963175, 0.970592778875, 0.981109691551, 0.957166949753},
    {0.109861750621, 0.485375648364, 0.798105856637, 0.800280473195, 0.297029449372, 0.141886345111}};

  for (const auto& wrenchValue : wrenchValues)
  {
    geometry_msgs::Wrench wrench;
    wrench.force.x = wrenchValue[0];
    wrench.force.y = wrenchValue[1];
    wrench.force.z = wrenchValue[2];
    wrench.torque.x = wrenchValue[3];
    wrench.torque.y = wrenchValue[4];
    wrench.torque.z = wrenchValue[5];

    const auto toRw = caros::toRw(wrench);
    EXPECT_DOUBLE_EQ(wrench.force.x, toRw(0));
    EXPECT_DOUBLE_EQ(wrench.force.y, toRw(1));
    EXPECT_DOUBLE_EQ(wrench.force.z, toRw(2));
    EXPECT_DOUBLE_EQ(wrench.torque.x, toRw(3));
    EXPECT_DOUBLE_EQ(wrench.torque.y, toRw(4));
    EXPECT_DOUBLE_EQ(wrench.torque.z, toRw(5));

    /* Forth and back */
    const auto toRwAndBack = caros::toRos(caros::toRw(wrench));
    EXPECT_DOUBLE_EQ(wrench.force.x, toRwAndBack.force.x);
    EXPECT_DOUBLE_EQ(wrench.force.y, toRwAndBack.force.y);
    EXPECT_DOUBLE_EQ(wrench.force.z, toRwAndBack.force.z);
    EXPECT_DOUBLE_EQ(wrench.torque.x, toRwAndBack.torque.x);
    EXPECT_DOUBLE_EQ(wrench.torque.y, toRwAndBack.torque.y);
    EXPECT_DOUBLE_EQ(wrench.torque.z, toRwAndBack.torque.z);
  }
}

/************************************************************************
 * Twist / VelocityScrew
 ************************************************************************/
TEST(TypeConversion, toRosFromRwVelocityScrew6D)
{
  const rw::math::VelocityScrew6D<> velocityScrews[] = {
    {0.814723691903, 0.135477004107, 0.905791934114, 0.835008589784, 0.126986811869, 0.968867771095},
    {0.913375855656, 0.22103404277, 0.632359249983, 0.30816705036, 0.0975404016208, 0.547220596345},
    {0.278498218395, 0.188381975982, 0.546881519025, 0.99288130179, 0.95750682964, 0.996461325325},
    {0.96488853381, 0.967694936786, 0.157613076502, 0.725838963175, 0.970592778875, 0.981109691551},
    {0.957166949753, 0.109861750621, 0.485375648364, 0.798105856637, 0.800280473195, 0.297029449372}};

  for (const auto& velocityScrew : velocityScrews)
  {
    const auto toRos = caros::toRos(velocityScrew);
    EXPECT_DOUBLE_EQ(velocityScrew(0), toRos.linear.x);
    EXPECT_DOUBLE_EQ(velocityScrew(1), toRos.linear.y);
    EXPECT_DOUBLE_EQ(velocityScrew(2), toRos.linear.z);
    EXPECT_DOUBLE_EQ(velocityScrew(3), toRos.angular.x);
    EXPECT_DOUBLE_EQ(velocityScrew(4), toRos.angular.y);
    EXPECT_DOUBLE_EQ(velocityScrew(5), toRos.angular.z);

    /* Forth and back */
    const auto toRosAndBack = caros::toRw(caros::toRos(velocityScrew));
    EXPECT_DOUBLE_EQ(velocityScrew(0), toRosAndBack(0));
    EXPECT_DOUBLE_EQ(velocityScrew(1), toRosAndBack(1));
    EXPECT_DOUBLE_EQ(velocityScrew(2), toRosAndBack(2));
    EXPECT_DOUBLE_EQ(velocityScrew(3), toRosAndBack(3));
    EXPECT_DOUBLE_EQ(velocityScrew(4), toRosAndBack(4));
    EXPECT_DOUBLE_EQ(velocityScrew(5), toRosAndBack(5));
  }
}

TEST(TypeConversion, toRwFromRosTwist)
{
  const double velocityScrewValues[][6] = {
    {0.814723691903, 0.135477004107, 0.905791934114, 0.835008589784, 0.126986811869, 0.968867771095},
    {0.913375855656, 0.22103404277, 0.632359249983, 0.30816705036, 0.0975404016208, 0.547220596345},
    {0.278498218395, 0.188381975982, 0.546881519025, 0.99288130179, 0.95750682964, 0.996461325325},
    {0.96488853381, 0.967694936786, 0.157613076502, 0.725838963175, 0.970592778875, 0.981109691551},
    {0.957166949753, 0.109861750621, 0.485375648364, 0.798105856637, 0.800280473195, 0.297029449372}};

  for (const auto& velocityScrewValue : velocityScrewValues)
  {
    geometry_msgs::Twist twist;
    twist.linear.x = velocityScrewValue[0];
    twist.linear.y = velocityScrewValue[1];
    twist.linear.z = velocityScrewValue[2];
    twist.angular.x = velocityScrewValue[3];
    twist.angular.y = velocityScrewValue[4];
    twist.angular.z = velocityScrewValue[5];

    const auto toRw = caros::toRw(twist);
    EXPECT_DOUBLE_EQ(twist.linear.x, toRw(0));
    EXPECT_DOUBLE_EQ(twist.linear.y, toRw(1));
    EXPECT_DOUBLE_EQ(twist.linear.z, toRw(2));
    EXPECT_DOUBLE_EQ(twist.angular.x, toRw(3));
    EXPECT_DOUBLE_EQ(twist.angular.y, toRw(4));
    EXPECT_DOUBLE_EQ(twist.angular.z, toRw(5));

    /* Forth and back */
    const auto toRwAndBack = caros::toRos(caros::toRw(twist));
    EXPECT_DOUBLE_EQ(twist.linear.x, toRwAndBack.linear.x);
    EXPECT_DOUBLE_EQ(twist.linear.y, toRwAndBack.linear.y);
    EXPECT_DOUBLE_EQ(twist.linear.z, toRwAndBack.linear.z);
    EXPECT_DOUBLE_EQ(twist.angular.x, toRwAndBack.angular.x);
    EXPECT_DOUBLE_EQ(twist.angular.y, toRwAndBack.angular.y);
    EXPECT_DOUBLE_EQ(twist.angular.z, toRwAndBack.angular.z);
  }
}

/************************************************************************
 * Float
 ************************************************************************/
TEST(TypeConversion, toRosFromFloat)
{
  const float floats[] = {0.8147, 0.1354, 0.9057, 0.8350, 0.1269, 0.9688, 0.9133, 0.2210, 0.6323, 0.3081};

  for (const auto f : floats)
  {
    const auto toRos = caros::toRos(f);
    EXPECT_DOUBLE_EQ(f, toRos);
  }
}

TEST(TypeConversion, toRwFromFloat)
{
  const float floats[] = {0.8147, 0.1354, 0.9057, 0.8350, 0.1269, 0.9688, 0.9133, 0.2210, 0.6323, 0.3081};

  for (const auto f : floats)
  {
    const auto toRw = caros::toRw(f);
    EXPECT_DOUBLE_EQ(f, toRw);
  }
}

/************************************************************************
 * Double
 ************************************************************************/
TEST(TypeConversion, toRosFromDouble)
{
  const double doubles[] = {0.0975404016208, 0.547220596345, 0.278498218395, 0.188381975982, 0.546881519025, 0.99288130179, 0.95750682964, 0.996461325325, 0.96488853381, 0.967694936786};

  for (const auto d : doubles)
  {
    const auto toRos = caros::toRos(d);
    EXPECT_DOUBLE_EQ(d, toRos);
  }
}

TEST(TypeConversion, toRwFromDouble)
{
  const double doubles[] = {0.0975404016208, 0.547220596345, 0.278498218395, 0.188381975982, 0.546881519025, 0.99288130179, 0.95750682964, 0.996461325325, 0.96488853381, 0.967694936786};

  for (const auto d : doubles)
  {
    const auto toRw = caros::toRw(d);
    EXPECT_DOUBLE_EQ(d, toRw);
  }
}

/************************************************************************
 * Bool
 ************************************************************************/
TEST(TypeConversion, toRosFromBool)
{
  const bool bools[] = {true, false};

  for (const auto d : bools)
  {
    const auto toRos = caros::toRos(d);
    EXPECT_EQ(d, toRos);
  }
}

TEST(TypeConversion, toRwFromBool)
{
  const bool bools[] = {true, false};

  for (const auto d : bools)
  {
    const auto toRw = caros::toRw(d);
    EXPECT_EQ(d, toRw);
  }
}

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
