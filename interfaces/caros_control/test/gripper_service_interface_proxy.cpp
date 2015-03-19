#include <caros/GripperSIProxy.hpp>
#include "gripper_service_interface_dummy.h"
#include <ros/ros.h>
#include <gtest/gtest.h>

#include <string>
#include <unordered_map>
#include <functional>
#include <tuple>
#include <memory>

typedef std::unordered_map<std::string, std::tuple<std::function<bool(caros::GripperSIProxy&)>, const std::string>> services_t;

namespace
{
const services_t servicesToTest = {
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
  }
};

/************************************************************************
 * Testor
 * Functions that perform the actual tests
 ************************************************************************/
struct TestorConfiguration
{
  std::function<bool()> func;
  std::string expectedFunctionCalled;
  bool expectedReturnValue;
  std::function<const std::string&()> getFunctionCalledFunc;
};
// Typedef for the signature of a testor function
typedef std::function<void(const TestorConfiguration&)> testor_t;

void testorReturnValue(const TestorConfiguration& conf)
{
  bool actualReturnValue = false;
  EXPECT_NO_THROW(actualReturnValue = conf.func());

  EXPECT_EQ(conf.expectedReturnValue, actualReturnValue);

  EXPECT_EQ(conf.expectedFunctionCalled, conf.getFunctionCalledFunc());
}

/* EXPECT_THROW doesn't work well with templated exception types, as it outputs the literal name of the exception identifier that is specified */
void testorUnavailableService(const TestorConfiguration& conf)
{
  EXPECT_THROW(conf.func(), caros::unavailableService);
}

void testorBadServiceCall(const TestorConfiguration& conf)
{
  EXPECT_THROW(conf.func(), caros::badServiceCall);
}

/************************************************************************
 * Test wrapper
 * (and services iterator)
 ************************************************************************/
struct TestWrapperConfiguration
{
  bool returnValueToTest;
  bool causeError;
  bool useServiceInterfaceDummy;
  testor_t testFunc;
};

/* D: service interface dummy
 * P: service interface proxy
 */
template<typename D, typename P>
void testWrapper(const services_t& services, const TestWrapperConfiguration& conf)
{
  ros::NodeHandle nodehandleService("si_dummy");
  std::shared_ptr<D> si_dummy(nullptr);
  if (conf.useServiceInterfaceDummy)
  {
    // Create the service interface dummy with the required parameters/configuration
    si_dummy = std::make_shared<D>(nodehandleService, conf.returnValueToTest, conf.causeError);
  }

  // Spawn 1 spinner thread
  ros::AsyncSpinner spinner(1);
  ASSERT_TRUE(spinner.canStart());
  spinner.start();

  ros::NodeHandle nodehandleClient("sip");
  P sip(nodehandleClient, "si_dummy");

  // The (unordered)map foreach uses member .first to access the key and .second to access the value
  for (const auto& service : services)
  {
    TestorConfiguration testorConf;
    testorConf.expectedReturnValue = conf.returnValueToTest;
    testorConf.expectedFunctionCalled = std::get<1>(service.second);

    // Bind the SIProxy member function to be called on the sip object (std::bind will implicitly make a copy of the sip object, so using std::ref to make a copy of a reference to the object - invoking the function on the original sip object)
    auto serviceFunc = std::get<0>(service.second);
    testorConf.func = std::bind(serviceFunc, std::ref(sip));

    // See comment for previous std::bind and std::ref (however not necessary if using a shared_ptr, as that can easily be copied and still point to the same object)
    testorConf.getFunctionCalledFunc = std::bind(&D::getMostRecentFunctionCalled, si_dummy);

    conf.testFunc(testorConf);
  }

  /* End */
  nodehandleClient.shutdown();
  nodehandleService.shutdown();

  spinner.stop();
}

/************************************************************************
 * Convenience functions
 ************************************************************************/
enum class TestType { ReturnTrue, ReturnFalse, BadServiceCall, UnavailableService };

TestWrapperConfiguration createTestConfiguration(const TestType testType)
{
  TestWrapperConfiguration conf;
  switch (testType)
  {
  case TestType::ReturnTrue:
    conf.returnValueToTest = true;
    conf.causeError = false;
    conf.useServiceInterfaceDummy = true;
    conf.testFunc = std::bind(testorReturnValue, std::placeholders::_1);
    break;
  case TestType::ReturnFalse:
    conf.returnValueToTest = false;
    conf.causeError = false;
    conf.useServiceInterfaceDummy = true;
    conf.testFunc = std::bind(testorReturnValue, std::placeholders::_1);
    break;
  case TestType::BadServiceCall:
    conf.returnValueToTest = false;
    conf.causeError = true;
    conf.useServiceInterfaceDummy = true;
    conf.testFunc = std::bind(testorBadServiceCall, std::placeholders::_1);
    break;
  case TestType::UnavailableService:
    conf.returnValueToTest = false;
    conf.causeError = false;
    conf.useServiceInterfaceDummy = false;
    conf.testFunc = std::bind(testorUnavailableService, std::placeholders::_1);
    break;
  default:
    throw std::runtime_error("Unsupported TestType enum!");
  }

  return conf;
}

/* D: service interface dummy
 * P: service interface proxy
 */
template<typename D, typename P>
void testServices(const services_t& services, const TestType testType)
{
  auto conf = createTestConfiguration(testType);

  testWrapper<D, P>(services, conf);
}

} // end namespace

TEST(GripperSIProxy, servicesSuccess)
{
  testServices<GripperServiceInterfaceDummy, caros::GripperSIProxy>(servicesToTest, TestType::ReturnTrue);
}

TEST(GripperSIProxy, servicesFailure)
{
  testServices<GripperServiceInterfaceDummy, caros::GripperSIProxy>(servicesToTest, TestType::ReturnFalse);
}
#if 0
TEST(GripperSIProxy, unavailableService)
{
  /* Overview:
   * No si_dummy
   * Call functions (catching the expected exception  - and verify that the value/text is as expected?)
   * Don't verify the function called (as no function was called) / or expect it to be false!
   */

  std::function<void(std::function<bool()>)> func = std::bind(testExceptionUnavailableService, std::placeholders::_1);
  testExceptionWrapper(func);
}

TEST(GripperSIProxy, badServiceCall)
{
  /* TODO:
   * Need the gripper_service_interface_dummy to screw up and cause he GripperSIProxy to throw badServiceCall
   */
  std::function<void(std::function<bool()>)> func = std::bind(testExceptionBadServiceCall, std::placeholders::_1);
  testExceptionWrapper(func);
}
#endif
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "gripper_service_interface_proxy");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
