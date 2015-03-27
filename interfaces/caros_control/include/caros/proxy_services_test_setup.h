#ifndef CAROS_CONTROL_PROXY_TEST_SETUP_H
#define CAROS_CONTROL_PROXY_TEST_SETUP_H

#include <caros/exceptions.h>

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <string>
#include <functional>
#include <tuple>
#include <memory>

namespace caros
{
namespace test
{
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

/* EXPECT_THROW doesn't work well with templated exception types, as it outputs the literal name of the exception
 * identifier that is specified */
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
 * C: container holding services
 */
template <typename D, typename P, typename C>
void testWrapper(const C& services, const TestWrapperConfiguration& conf)
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

  for (const auto& service : services)
  {
    TestorConfiguration testorConf;
    testorConf.expectedReturnValue = conf.returnValueToTest;
    testorConf.expectedFunctionCalled = std::get<1>(service);

    // Bind the SIProxy member function to be called on the sip object (std::bind will implicitly make a copy of the sip
    // object, so using std::ref to make a copy of a reference to the object - invoking the function on the original sip
    // object)
    auto serviceFunc = std::get<0>(service);
    testorConf.func = std::bind(serviceFunc, std::ref(sip));

    // See comment for previous std::bind and std::ref (however not necessary if using a shared_ptr, as that can easily
    // be copied and still point to the same object)
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
enum class TestType
{
  ReturnTrue,
  ReturnFalse,
  BadServiceCall,
  UnavailableService
};

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
 * C: container holding services
 */
template <typename D, typename P, typename C>
void testServices(const C& services, const TestType testType)
{
  auto conf = createTestConfiguration(testType);

  testWrapper<D, P, C>(services, conf);
}
}  // end namespace
}  // end namespace

#endif
