#include <caros/universal_robots.h>

#include <caros/common.h>
#include <caros/common_robwork.h>

#include <rw/math/MetricFactory.hpp>

#include <ros/assert.h>

#include <string>

using namespace caros;

#define URRT_PORT 30003
#define UR_PORT 30001
#define WRENCHTOPIC_QUEUE_SIZE 5
#define WRENCH_DATA_QUEUE_MAX_ALLOWED_NUMBER_OF_ELEMENTS 3
#define URRTDATA_QACTUAL_SIZE 6

UniversalRobots::UniversalRobots(const ros::NodeHandle& nodehandle)
    : CarosNodeServiceInterface(nodehandle),
      SerialDeviceServiceInterface(nodehandle),
      URServiceInterface(nodehandle),
      nodehandle_(nodehandle),
      workcell_(NULL),
      device_(NULL),
      ft_frame_(NULL),
      use_ft_collision_detection_(false)
{
  /* Currently nothing specific should happen */
}

UniversalRobots::~UniversalRobots()
{
  /* Nothing special to destroy */
}

bool UniversalRobots::activateHook()
{
  /* TODO:
   * - Maybe split all this configuration/activation logic up into a configure and connect step, like what has been done
   * for the SDH node.
   * ^- Should the fetching of the parameters be moved into its own function, instead of having this big activateHook()
   * function?
   */
  /************************************************************************
   * Parameters
   ************************************************************************/
  std::string device_name;
  if (not nodehandle_.getParam("device_name", device_name))
  {
    CAROS_FATALERROR("The parameter '" << nodehandle_.getNamespace() << "/device_name' was not present on the parameter "
                                                                        "server! This parameter has to be specified "
                                                                        "for this node to work properly.",
                     URNODE_MISSING_PARAMETER);

    return false;
  }

  std::string ft_frame_name;
  nodehandle_.param("ft_frame", ft_frame_name, std::string("WORLD"));

  std::string device_ip;
  if (not nodehandle_.getParam("device_ip", device_ip))
  {
    CAROS_FATALERROR("The parameter '" << nodehandle_.getNamespace() << "/device_ip' was not present on the parameter server! "
                                                                        "This parameter has to be specified for this "
                                                                        "node to work properly.",
                     URNODE_MISSING_PARAMETER);
    return false;
  }

  std::string callback_ip;
  if (not nodehandle_.getParam("callback_ip", callback_ip))
  {
    CAROS_FATALERROR("The parameter '" << nodehandle_.getNamespace() << "/callback_ip' was not present on the parameter "
                                                                        "server! This parameter has to be specified "
                                                                        "for this node to work properly.",
                     URNODE_MISSING_PARAMETER);
    return false;
  }

  std::string callback_port;
  if (not nodehandle_.getParam("callback_port", callback_port))
  {
    CAROS_FATALERROR("The parameter '" << nodehandle_.getNamespace() << "/callback_port' was not present on the "
                                                                        "parameter server! This parameter has to be "
                                                                        "specified for this node to work properly.",
                     URNODE_MISSING_PARAMETER);
    return false;
  }

  std::string wrench_topic;
  nodehandle_.param("wrench_topic", wrench_topic, std::string());
  if (not wrench_topic.empty())
  {
    sub_ft_data_ = nodehandle_.subscribe(wrench_topic, WRENCHTOPIC_QUEUE_SIZE, &UniversalRobots::addFTData, this);
    if (not sub_ft_data_)
    {
      CAROS_FATALERROR("The subscriber for the topic '" << wrench_topic << "' could not be properly created.",
                       URNODE_FAULTY_SUBSCRIBER);
      return false;
    }
  }

  workcell_ = caros::getWorkCell();
  if (workcell_ == NULL)
  {
    CAROS_FATALERROR("No workcell was provided!", URNODE_MISSING_WORKCELL);
    return false;
  }

  ROS_ASSERT(workcell_ != NULL);
  ROS_DEBUG_STREAM("Looking for the device '" << device_name << "' in the workcell.");
  device_ = workcell_->findDevice(device_name);
  if (device_ == NULL)
  {
    CAROS_FATALERROR("Unable to find device " << device_name << " in the loaded workcell",
                     URNODE_NO_SUCH_DEVICE_IN_WORKCELL);
    return false;
  }

  ROS_DEBUG_STREAM("Looking for the frame '" << ft_frame_name << "' in the workcell.");
  ft_frame_ = workcell_->findFrame(ft_frame_name);
  if (ft_frame_ == NULL)
  {
    /* Just a warning since only a single function makes use of the frame.
     * However currently it requires a node restart to reinitialise the parameters, workcell and finding the ft_frame.
     */
    ROS_WARN_STREAM("Couldn't find the FT Frame '" << ft_frame_name << "' in the workcell!");
    return false;
  }

  state_ = workcell_->getDefaultState();
  ik_solver_ = rw::common::ownedPtr(new rw::invkin::JacobianIKSolver(device_, state_));
  ROS_ASSERT(ik_solver_ != NULL);

  try
  {
    urrt_.connect(device_ip, URRT_PORT);
  }
  catch (rw::common::Exception& exp)
  {
    CAROS_FATALERROR("Could not connect to urrt:" << exp.what(), URNODE_UNSUCCESSFUL_CONNECT_TO_URRT);
    return false;
  }

  try
  {
    ur_.connect(device_ip, UR_PORT);
  }
  catch (rw::common::Exception& exp)
  {
    CAROS_FATALERROR("Could not connect to urrt:" << exp.what(), URNODE_UNSUCCESSFUL_CONNECT_TO_UR);
    return false;
  }

  unsigned int numeric_callback_port = 0;
  try
  {
    numeric_callback_port = std::stoul(callback_port);
  }
  catch (std::exception& e)
  {
    CAROS_FATALERROR("The supplied callback port could not be converted to a numeric value.",
                     URNODE_INVALID_CALLBACKPORT);
    return false;
  }

  /* The order of starting ur_ and urrt_ doesn't seem to matter */
  /* No feedback from startCommunication() ... there are debug log messages on whether it was sort of successful or not
   */
  ur_.startCommunication(callback_ip, numeric_callback_port);
  /* No feedback from start() */
  urrt_.start();

  if (not URServiceInterface::configureInterface())
  {
    CAROS_FATALERROR("The URService could not be configured correctly.", URNODE_URSERVICE_CONFIGURE_FAIL);
    return false;
  }

  if (not SerialDeviceServiceInterface::configureInterface())
  {
    CAROS_FATALERROR("The SerialDeviceService could not be configured correctly.",
                     URNODE_SERIALDEVICESERVICE_CONFIGURE_FAIL);
    return false;
  }

  return true;
}

bool UniversalRobots::recoverHook(const std::string& error_msg, const int64_t error_code)
{
  /* TODO: Missing handling all the different scenarios. */
  /* TODO: Go into fatal error if recover is invoked on the unsupported error scenarios */
  bool resolved = false;

  switch (error_code)
  {
    case URNODE_UNSUPPORTED_Q_LENGTH:
      /* Simply acknowledge that a wrong Q was provided */
      resolved = true;
      break;
    case URNODE_INTERNAL_ERROR:
      CAROS_FATALERROR("Can not resolve an internal error... ending up in this case/situation is a bug!",
                       URNODE_INTERNAL_ERROR);
      break;
    default:
      CAROS_FATALERROR("The provided error code '"
                           << error_code << "' has no recovery functionality! - this should be considered a bug!",
                       URNODE_INTERNAL_ERROR);
      break;
  }

  return resolved;
}

void UniversalRobots::runLoopHook()
{
  bool urrt_has_data = urrt_.hasData();
  bool ur_has_data = ur_.getPrimaryInterface().hasData();

  if (urrt_has_data == false || ur_has_data == false)
  {
    ROS_DEBUG_STREAM("Missing some data from UR: urrt_has_data = '" << urrt_has_data << "' and ur_has_data = '"
                                                                    << ur_has_data << "'");
    return;
  }

  rwhw::URRTData urrt_data = urrt_.getLastData();
  rwhw::UniversalRobotsData pur_data = ur_.getPrimaryInterface().getLastData();

  ROS_DEBUG_STREAM("E-stop: '" << pur_data.emergencyStopped << "' Security-stop: '" << pur_data.securityStopped
                               << "' Running: '" << pur_data.programRunning << "'");

  auto messages = ur_.getPrimaryInterface().getMessages();
  while (not messages.empty())
  {
    ROS_DEBUG_STREAM("UR Message: " << messages.front());
    messages.pop();
  }
  ur_.getPrimaryInterface().clearMessages();

  if (urrt_data.qActual.size() == URRTDATA_QACTUAL_SIZE)
  {
    caros_control_msgs::RobotState robot_state;
    qcurrent_ = urrt_data.qActual;
    robot_state.q = caros::toRos(urrt_data.qActual);
    robot_state.dq = caros::toRos(urrt_data.dqActual);
    robot_state.header.frame_id = nodehandle_.getNamespace();
    robot_state.header.stamp = ros::Time::now();
    robot_state.e_stopped = caros::toRos(pur_data.emergencyStopped);

    /* TODO: Currently there is a delay somewhere, where the data gotten from the robot is really delayed quite a bit -
     * atleast for the emergency stop. */

    /* TODO: This isMoving() function is not working - returns false eventhough the robot is moving...
     * isMoving() is returning a variable that is modified within a thread - Perhaps the compiler has optimised the read
     * to be constant? */
    robot_state.is_moving = caros::toRos(ur_.isMoving());

    /* TODO:
     * isColliding is hardcoded to be false....
     * ^- Look at the old out-commented code in the MARVIN version for hints on how the collision detection was done (if
     * it's desirable to bring back that functionality)
     */
    robot_state.is_colliding = caros::toRos(false);

    SerialDeviceServiceInterface::publishState(robot_state);
  }
  else
  {
    /* TODO:
     * Should this be an error (just ROS error or also a CAROS error - e.g. changing the state of this node to error or
     * fatalerror)?
     */
    ROS_WARN_STREAM("The size of urrt_data.qActual is '" << urrt_data.qActual.size() << "' but should be '"
                                                         << URRTDATA_QACTUAL_SIZE << "'!"
                                                         << " - Not publishing robot state!");
  }
}

void UniversalRobots::errorLoopHook()
{
  /* TODO:
   * Consider what needs to be done when this node is in error - should any of the urrt_ or ur_ objects/connections be
   * stopped or just let them continue?
   */
}

void UniversalRobots::fatalErrorLoopHook()
{
  /* TODO:
   * Consider what needs to be done when this node is in error - should any of the urrt_ or ur_ objects/connections be
   * stopped or just let them continue?
   */
}

void UniversalRobots::addFTData(const caros_common_msgs::WrenchData::ConstPtr state)
{
  rw::math::Wrench6D<> wrench;
  wrench(0) = state->wrench.force.x;
  wrench(1) = state->wrench.force.y;
  wrench(2) = state->wrench.force.z;
  wrench(3) = state->wrench.torque.x;
  wrench(4) = state->wrench.torque.y;
  wrench(5) = state->wrench.torque.z;

  if (wrench_data_queue_.size() > WRENCH_DATA_QUEUE_MAX_ALLOWED_NUMBER_OF_ELEMENTS)
  {
    wrench_data_queue_.pop();
  }

  wrench_data_queue_.push(wrench);
}

/************************************************************************
 * URServiceInterface functions
 ************************************************************************/
bool UniversalRobots::urServoT(const rw::math::Transform3D<>& target)
{
  ROS_DEBUG_STREAM("servoT: " << target);

  if (not isInWorkingCondition())
  {
    return false;
  }

  device_->setQ(qcurrent_, state_);
  std::vector<rw::math::Q> solutions = ik_solver_->solve(target, state_);
  if (solutions.empty())
  {
    ROS_ERROR_STREAM("servoT: Unable to find IK solution for target = '" << target << "' and qcurrent_ = '" << qcurrent_
                                                                         << "'");
    return false;
  }

  rw::math::Q closest = solutions.front();

  if (not supportedQSize(closest))
  {
    return false;
  }

  ROS_DEBUG_STREAM("servoT: Q-configuration being sent to driver: " << closest);
  ur_.servo(closest);
  /* There is no (immediate) feedback from the ur_.servo() function call, so just returning true. */
  return true;
}

bool UniversalRobots::urServoQ(const rw::math::Q& q)
{
  ROS_DEBUG_STREAM("ServoQ: " << q);

  if (not isInWorkingCondition() || not supportedQSize(q))
  {
    return false;
  }

  ur_.servo(q);
  /* There is no (immediate) feedback from the ur_.servo() function call, so just returning true. */
  return true;
}

bool UniversalRobots::urForceModeStart(const rw::math::Transform3D<>& refToffset, const rw::math::Q& selection,
                                       const rw::math::Wrench6D<>& wrench_target, const rw::math::Q& limits)
{
  ROS_DEBUG_STREAM("ForceModeStart arguments begin:");
  ROS_DEBUG_STREAM("refToffset: " << refToffset);
  ROS_DEBUG_STREAM("selection: " << selection);
  ROS_DEBUG_STREAM("wrench_target: " << wrench_target);
  ROS_DEBUG_STREAM("limits: " << limits);
  ROS_DEBUG_STREAM("ForceModeStart arguments end");

  if (not isInWorkingCondition())
  {
    return false;
  }

  if (selection.size() != 6)
  {
    ROS_WARN_STREAM("The number of elements in selection is '" << selection.size() << "' but should be '" << 6 << "'");
    return false;
  }
  if (limits.size() != 6)
  {
    ROS_WARN_STREAM("The number of elements in limits is '" << limits.size() << "' but should be '" << 6 << "'");
    return false;
  }

  ur_.forceModeStart(refToffset, selection, wrench_target, limits);
  /* There is no (immediate) feedback from the ur_.forceModeStart() function call, so just returning true. */
  return true;
}

bool UniversalRobots::urForceModeUpdate(const rw::math::Wrench6D<>& wrench_target)
{
  ROS_DEBUG_STREAM("New wrench target for forceModeUpdate: " << wrench_target);

  if (not isInWorkingCondition())
  {
    return false;
  }

  ur_.forceModeUpdate(wrench_target);
  return true;
}

bool UniversalRobots::urForceModeStop()
{
  if (not isInWorkingCondition())
  {
    return false;
  }

  ur_.forceModeEnd();

  return true;
}

/************************************************************************
 * SerialDeviceServiceInterface
 ************************************************************************/
bool UniversalRobots::moveLin(const TransformAndSpeedContainer_t& targets)
{
  ROS_DEBUG_STREAM("moveLin with " << targets.size() << " target(s).");

  if (not isInWorkingCondition())
  {
    return false;
  }

  for (const auto& target : targets)
  {
    /* moveT(...) is void, so no errorcode is returned. Furthermore the implementation (at least in revision 5472)
     * doesn't make use of the speeds. */
    ur_.moveT(std::get<0>(target), std::get<1>(target)); /* This could be rewritten to be more explicit about the
                                                            parameters, such that std::get<1> is the speed */
  }

  /* Simply return true, given there is no feedback from the ur_.moveT(...) function */
  return true;
}

bool UniversalRobots::movePtp(const QAndSpeedContainer_t& targets)
{
  ROS_DEBUG_STREAM("movePtp with " << targets.size() << " target(s).");

  if (not isInWorkingCondition())
  {
    return false;
  }

  for (const auto& target : targets)
  {
    const auto& q = std::get<0>(target);
    if (not supportedQSize(q))
    {
      return false;
    }

    ur_.moveQ(q, std::get<1>(target));
  }

  return true;
}

bool UniversalRobots::movePtpT(const TransformAndSpeedContainer_t& targets)
{
  ROS_DEBUG_STREAM("movePtpT with " << targets.size() << " target(s).");

  if (not isInWorkingCondition())
  {
    return false;
  }

  for (const auto& target : targets)
  {
    device_->setQ(qcurrent_, state_);
    rw::math::Transform3D<> transform = std::get<0>(target);
    std::vector<rw::math::Q> solutions = ik_solver_->solve(transform, state_);
    if (solutions.empty())
    {
      ROS_WARN_STREAM("movePtpT: Unable to find IK solution for: " << transform << " with qcurrent: " << qcurrent_);
      return false;
    }

    const rw::math::Q& q = solutions.front();
    if (not supportedQSize(q))
    {
      return false;
    }

    ur_.moveQ(q, std::get<1>(target));
  }

  return true;
}

/* Forwarding the movement to the URServiceInterface function servoQ */
bool UniversalRobots::moveVelQ(const rw::math::Q& q_vel)
{
  ROS_ERROR_STREAM("Current implementation does not follow the specification!");
  return false;

/******************************
 * Old Implementation
 ******************************/
#if 0
  if (not isInWorkingCondition() || not supportedQSize(q_vel))
  {
    return false;
  }

    /* TODO:
     * Missing documentation on why the factor 0.1 is used and not some other arbitrary value?
     * And 1/10th of the value is added directly to the current joint values/angles, making a q-value of 0-100 (%) up to 10 radians, which is quite a lot - I doubt that this was the intension when it got implemented in MARVIN...
     */
    return urServoQ(qcurrent_ + q_vel*0.1);
#endif
}

/* Forwarding the movement to the URServiceInterface function servoQ */
bool UniversalRobots::moveVelT(const rw::math::VelocityScrew6D<>& t_vel)
{
  ROS_ERROR_STREAM("Current implementation has not been verified to follow the specification!");
  return false;

/******************************
 * Old Implementation
 ******************************/
#if 0
  if (not isInWorkingCondition())
  {
    return false;
  }

	device_->setQ(qcurrent_, state_);
	rw::math::Jacobian jac = device_->baseJend(state_);
        /* TODO:
         * Find out where the 'Eigen' got introduced from, since it doesn't exist...
         *	rw::math::Jacobian jac_inv( rw::math::LinearAlgebra::pseudoInverseEigen(jac.e()) );
         */
	rw::math::Jacobian jac_inv( rw::math::LinearAlgebra::pseudoInverse(jac.e()) );

	/* TODO:
         * Could use some more documentation on why the factor of 0.1 is being used (see todo comment for moveVelQ)
         */
        rw::math::Q qtarget = qcurrent_ + (jac_inv*t_vel)*0.1;
	return urServoQ(qtarget);
#endif
}

/* Forwarding the movement to the URServiceInterface function servoT */
bool UniversalRobots::moveLinFc(const rw::math::Transform3D<>& pos_target, const rw::math::Transform3D<>& offset,
                                const rw::math::Wrench6D<>& wrench_target, const rw::math::Q& control_gain)
{
  ROS_ERROR_STREAM(
      "Current implementation has not been verified to follow the specification nor been tested in CAROS!");
  return false;

/******************************
 * Old Implementation
 ******************************/
#if 0
  if (not isInWorkingCondition())
  {
    return false;
  }

    if (ft_frame_ == NULL) {
      /* It is also possible to go into an error state due to the missing ft_frame, and have a recover scenario handle fetching a the ft_frame name from the parameter server and try to find it in the workcell again. */
        ROS_WARN_STREAM("Unable to use force command without having defined a FT frame for the sensor.");
        return false;
    }

    rw::kinematics::State state = workcell_->getDefaultState();
    device_->setQ(qcurrent_, state);

    rw::math::Transform3D<> baseTref = rw::math::Transform3D<>::identity();
    rw::math::Transform3D<> baseTtarget = baseTref*pos_target;
    rw::math::Transform3D<> baseToffset = baseTref*offset;
    rw::math::Transform3D<> base2ft = device_->baseTframe(ft_frame_, state);
    rw::math::Transform3D<> baseTend = device_->baseTend(state);
    rw::math::Transform3D<> endTtarget = inverse(baseTend)*baseTtarget;
    rw::math::Transform3D<> endToffset = inverse(baseTend)*baseToffset;
    rw::math::Transform3D<> ftToffset = inverse(base2ft)*baseToffset;

    /* TODO:
     * If control_gain consist of elements that are all 0 then the wrench_data_queue_ element will not be used anyway and could be a case that should still work even when no or not enough wrench data is being published?
     *
     * FIXME: TODO:
     * Should look at the timestamps of the wrench data that has been received and verify that the received wrench data is within the allowed timestamp difference - not too old (or newer - should be caught in a ROS_WARN / test <- should this actually be an error or fatal error - maybe even an assert? as something really bogus could be going on...)
     * ^- Maybe it would make sense to function-locally(i.e. static) save the last used wrench timestamp to make sure that older samples in the queue are not being used (but then they will still remain there to be used for averaging/mean sampling.
     */
    if (wrench_data_queue_.empty()) {
        ROS_WARN_STREAM("There have been no new wrench data coming from the ROS topic '" << sub_ft_data_.getTopic() << "'");
        return false;
    }
    ROS_ASSERT(not wrench_data_queue_.empty());
    /* Do not alter the wrench_data_queue_ - just make a copy of the newest element */
    rw::math::Wrench6D<> wrench_current = wrench_data_queue_.back();

    wrench_current.setForce(ftToffset.R() * wrench_current.force());
    wrench_current.setTorque(ftToffset.R() * wrench_current.torque());

    rw::math::Wrench6D<> wrench_diff (wrench_target.force() - wrench_current.force(), wrench_target.torque() - wrench_current.torque());

    rw::math::Vector3D<> pos_offset = endToffset.R() * endTtarget.P();
    rw::math::EAA<> eaa_offset = endToffset.R() * rw::math::EAA<>(endTtarget.R());

    /* Verify that the size of control_gain is 6 (i.e. the size of Wrench6D) */
    ROS_ASSERT(control_gain.size() == 6);
    for (unsigned int index = 0; index < 3; ++index) {
        pos_offset(index) = control_gain(index) * wrench_diff(index);
        eaa_offset(index) = control_gain(index+3) * wrench_diff(index+3);
    }

    endToffset = rw::math::Transform3D<>(pos_offset, eaa_offset);

    rw::math::Transform3D<> baseTtarget2 = baseTend * endToffset;

    return urServoT(baseTtarget2);
#endif
}

bool UniversalRobots::moveServoQ(const QAndSpeedContainer_t& targets)
{
  if (not isInWorkingCondition())
  {
    return false;
  }

  /* Throwing away the speed:
   * RobWorkHardware doesn't support specifying the speed when servoing
   */
  bool res = false;
  for (const auto& target : targets)
  {
    const auto& q = std::get<0>(target);
    if (not supportedQSize(q))
    {
      return false;
    }

    res = urServoQ(q);
    if (not res)
    {
      break;
    }
  }

  return res;
}

bool UniversalRobots::moveServoT(const TransformAndSpeedContainer_t& targets)
{
  if (not isInWorkingCondition())
  {
    return false;
  }

  bool res = false;
  /* Throwing away the speed */
  for (const auto& target : targets)
  {
    res = urServoT(std::get<0>(target));
    if (not res)
    {
      break;
    }
  }

  return res;
}

bool UniversalRobots::moveStart()
{
  if (not isInWorkingCondition())
  {
    return false;
  }

  ROS_ERROR_STREAM("Currently not implemented!");

  return false;
}

bool UniversalRobots::moveStop()
{
  if (not isInWorkingCondition())
  {
    return false;
  }

  ur_.stopRobot();

  return true;
}

bool UniversalRobots::movePause()
{
  if (not isInWorkingCondition())
  {
    return false;
  }

  ROS_ERROR_STREAM("Currently not implemented!");
  return false;
}

bool UniversalRobots::moveSetSafeModeEnabled(const bool value)
{
  if (not isInWorkingCondition())
  {
    return false;
  }

  ROS_ERROR_STREAM("Currently not implemented!");
  return false;
}

/************************************************************************
 * Utility functions
 ************************************************************************/
bool UniversalRobots::isInWorkingCondition()
{
  if (not isInRunning())
  {
    ROS_WARN_STREAM("Not in running state!");
    return false;
  }

  return true;
}

bool UniversalRobots::supportedQSize(const rw::math::Q& q)
{
  if (q.size() != SUPPORTED_Q_LENGTH_FOR_UR)
  {
    CAROS_ERROR("The length of Q is " << q.size() << " but should be " << SUPPORTED_Q_LENGTH_FOR_UR,
                URNODE_UNSUPPORTED_Q_LENGTH);
    return false;
  }

  return true;
}
