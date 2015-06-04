#include <caros/serial_device_si_proxy.h>
#include <caros/common_robwork.h>

#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>

#include <ros/ros.h>

#include <string>
#include <stdexcept>

/* WARNING: USE AT YOUR OWN RISK!
 * None of this is code is guaranteed to not make the robot crash into anything or do unexpected behaviour!
 * It is highly recommended that you understand what the code does before you run it!
 *
 * To minimise risk, please use (very) small qChange values!
 */

/* This class is supposed to make it a bit easier to reuse the test code.
 * To use another planner or collision detector, simply add a member function that the user can invoke and override the
 * defaults (before invoking the test functions)
 */
class UrTest
{
 public:
  UrTest() : nodehandle_("~"), sdsip_(nodehandle_, "ur_simple_demo_node")
  {
    initWorkCell();
    initDevice();
    initPathPlannerWithCollisionDetector();
  }

  virtual ~UrTest(){/* Empty */
  };

  bool testMovePtp(const double qChange)
  {
    if (not doTestMovePtp(qChange))
    {
      return false;
    }
    ROS_INFO_STREAM("Waiting a little moment for the movement to be finished");
    ros::Duration(5).sleep();  // In seconds

    if (not doTestMovePtp(-qChange))
    {
      return false;
    }
    ROS_INFO_STREAM("Waiting a little moment for the movement to be finished");
    ros::Duration(5).sleep();  // In seconds

    return true;
  }

  bool testMoveServoQ(const double qChange)
  {
    if (not doTestMoveServoQ(qChange))
    {
      return false;
    }
    ROS_INFO_STREAM("Waiting a little moment for the movement to be finished");
    ros::Duration(5).sleep();  // In seconds

    if (not doTestMoveServoQ(-qChange))
    {
      return false;
    }
    ROS_INFO_STREAM("Waiting a little moment for the movement to be finished");
    ros::Duration(5).sleep();  // In seconds

    return true;
  }

 protected:
  void initWorkCell()
  {
    workcell_ = caros::getWorkCell();
    if (workcell_ == NULL)
    {
      ROS_ERROR("No workcell was loaded - exiting...");
      throw std::runtime_error("Not able to obtain a workcell.");
    }
  }

  void initDevice()
  {
    std::string deviceName;
    if (not nodehandle_.getParam("deviceName", deviceName))
    {
      ROS_FATAL_STREAM("The parameter '" << nodehandle_.getNamespace()
                                         << "/deviceName' was not present on the parameter "
                                            "server! This parameter has to be specified "
                                            "for this test-node to work properly.");
      throw std::runtime_error("Not able to obtain device name.");
    }

    ROS_DEBUG_STREAM("Looking for the device '" << deviceName << "' in the workcell.");
    device_ = workcell_->findDevice(deviceName);
    if (device_ == NULL)
    {
      ROS_FATAL_STREAM("Unable to find device " << deviceName << " in the loaded workcell");
      throw std::runtime_error("Not able to find the device within the workcell.");
    }
  }

  void initPathPlannerWithCollisionDetector()
  {
    rw::kinematics::State state = workcell_->getDefaultState();
    /* Collision detector */
    auto detector = rw::common::ownedPtr(
        new rw::proximity::CollisionDetector(workcell_, rwlibs::proximitystrategies::ProximityStrategyYaobi::make()));
    /* PlannerConstraint that uses the collision detector to verify that the _start_ and _end_ configurations are
     * collision free and that the edge(s) between those is/are also collision free. */
    const rw::pathplanning::PlannerConstraint plannerConstraint =
        rw::pathplanning::PlannerConstraint::make(detector, device_, state);

    /* Just using a really simple path planner (straight line in the configuration space) */
    planner_ = rw::pathplanning::QToQPlanner::make(plannerConstraint);
  }

  rw::math::Q getCurrentJointConfiguration()
  {
    /* Make sure to get and operate on fresh data from the serial device
     * It's assumed that the serial device is not moving
     * ^- That could be asserted / verified using sdsip.isMoving()
     * However other sources could invoke services on the UR that causes it to move...
     */
    ros::Time currentTimestamp = ros::Time::now();
    ros::Time obtainedTimestamp = sdsip_.getTimeStamp();
    while (currentTimestamp > obtainedTimestamp)
    {
      ros::Duration(0.1).sleep();  // In seconds
      ros::spinOnce();
      obtainedTimestamp = sdsip_.getTimeStamp();
    }

    return sdsip_.getQ();
  }

  rw::trajectory::QPath getQPath(const double qChange)
  {
    rw::math::Q startConfiguration = getCurrentJointConfiguration();
    rw::math::Q endConfiguration = startConfiguration + rw::math::Q(startConfiguration.size(), qChange);

    rw::trajectory::QPath path;
    bool validPath = false;
    ROS_ASSERT(planner_);
    validPath = planner_->query(startConfiguration, endConfiguration, path);

    if (not validPath)
    {
      ROS_ERROR_STREAM("Could not find a path from '" << startConfiguration << "' to '" << endConfiguration << "'.");
      throw std::runtime_error("No valid path found.");
    }

    return path;
  }

  rw::trajectory::QPath linearInterpolatedPath(const rw::math::Q& start, const rw::math::Q& end,
                                               const double totalDuration = 10.0, const double durationStep = 1.0)
  {
    ROS_ASSERT(durationStep > 0);
    ROS_ASSERT(durationStep < totalDuration);

    rw::trajectory::QLinearInterpolator interpolator(start, end, totalDuration);

    rw::trajectory::QPath path;

    path.push_back(start);
    for (double t = durationStep; t <= (totalDuration - durationStep); t += durationStep)
    {
      path.push_back(interpolator.x(t));
    }
    path.push_back(end);

    return path;
  }

  bool doTestMovePtp(const double qChange)
  {
    bool returnStatus = true;
    rw::trajectory::QPath path = getQPath(qChange);
    for (const rw::math::Q& p : path)
    {
      ROS_INFO_STREAM("Ask to movePtp to '" << p << "'.");
      bool ret = false;
      ret = sdsip_.movePtp(p);
      if (not ret)
      {
        returnStatus = false;
        ROS_ERROR_STREAM("The serial device didn't acknowledge the movePtp command.");
      }
    }

    return returnStatus;
  }

  bool doTestMoveServoQ(const double qChange)
  {
    bool returnStatus = true;
    rw::trajectory::QPath path = getQPath(qChange);

    ROS_ASSERT(path.size() == 2);
    rw::math::Q startConfiguration = path.at(0);
    rw::math::Q endConfiguration = path.at(1);

    // replace the path with an interpolated path
    path = linearInterpolatedPath(startConfiguration, endConfiguration);

    for (const rw::math::Q& p : path)
    {
      ROS_INFO_STREAM("Ask to moveServoQ to '" << p << "'.");
      bool ret = false;
      ret = sdsip_.moveServoQ(p);
      if (not ret)
      {
        returnStatus = false;
        ROS_ERROR_STREAM("The serial device didn't acknowledge the moveServoQ command.");
      }
    }

    return returnStatus;
  }

 protected:
  ros::NodeHandle nodehandle_;
  caros::SerialDeviceSIProxy sdsip_;

  rw::models::WorkCell::Ptr workcell_;
  rw::models::Device::Ptr device_;
  rw::pathplanning::QToQPlanner::Ptr planner_;
};
