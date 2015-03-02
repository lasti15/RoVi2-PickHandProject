#include <caros/common.h>
#include <gtest/gtest.h>

#if 0
TEST(TypeConversion, state)
{
  /* Unittest:
   * Make/get a state -> Just use the default state? (or else a workcell has to be provided and all the dependencies (i.e. included files such as the UR robots, environment, cameras and etc.)
   * create a clone/copy
   * change some values in the clone
   * use the clone to convert to ros type
   * then use the original state for converting from ros to rw
   * verify that this state is now completely equal to the clone (used for converting to ros).
   */
}
#endif
/************************************************************************
 * Q
 ************************************************************************/
TEST(TypeConversion, toRos_from_rw_q)
{
  const rw::math::Q rwQ(6, 3.0);
  const auto toRosQ = caros::toRos(rwQ);
  ASSERT_EQ(toRosQ.data.size(), rwQ.size());
  for (std::size_t index = 0; index < rwQ.size(); ++index)
  {
    EXPECT_DOUBLE_EQ(toRosQ.data.at(index), rwQ[index]);
  }
}

TEST(TypeConversion, toRw_from_ros_q)
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
}
/************************************************************************
 * Transform
 ************************************************************************/
TEST(TypeConversion, toRos_from_rw_transform3D)
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
  }
}

TEST(TypeConversion, toRw_from_ros_transform)
{
  // The comparisonPrecision value is found by trial and error.
  const double comparisonPrecision = 1.0e-05;
  geometry_msgs::Transform transform;

  /* 5 randomly generated Transform3D - could also do this with seeded Math::ranTransform3D calls */
  const rw::math::Transform3D<> rwTransforms[] = {
    {rw::math::Vector3D<>(0.821768, -0.294349, -0.48791), rw::math::Rotation3D<>(-0.182442, -0.931498, 0.314686, -0.810775, -0.0385244, -0.584088, 0.5562, -0.361702, -0.748207)},
    {rw::math::Vector3D<>(0.0947497, -0.968762, 0.229179), rw::math::Rotation3D<>(-0.149109, -0.717196, 0.680732, -0.765136, 0.519767, 0.380011, -0.626365, -0.46419, -0.626255)},
    {rw::math::Vector3D<>(-0.234126, -0.938533, 0.253656), rw::math::Rotation3D<>(-0.279391, 0.75009, 0.599421, 0.238234, -0.550607, 0.800048, 0.930153, 0.366328, -0.0248621)},
    {rw::math::Vector3D<>(-0.252482, -0.967311, 0.0237235), rw::math::Rotation3D<>(0.439868, 0.168623, 0.88209, 0.595296, -0.790167, -0.145804, 0.672412, 0.589239, -0.44795)},
    {rw::math::Vector3D<>(-0.779311, 0.626539, 0.0110964), rw::math::Rotation3D<>(-0.0360424, -0.433045, -0.900652, 0.948205, -0.299441, 0.10603, -0.315608, -0.850181, 0.421408)}};

  for (const auto& rwTransform : rwTransforms)
  {
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
  }

/* TODO:
 * Also do a test that simply converts to ros and back again and then verifies - testing both the to and from ros conversions
 */

}
/* Either test toRos and then fromRos
 * ^--> Or test the toRos and fromRos in one go where the rw::math::Transform3Ds are compared - however this could still lead to a faulty ros representation since that one is not being verified in the process - however it results in less testing oriented code...
 */

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
