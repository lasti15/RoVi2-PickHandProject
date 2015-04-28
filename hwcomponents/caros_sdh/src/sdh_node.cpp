#include <caros/sdh_node.h>

#include <caros/caros_node_service_interface.h>
#include <caros/gripper_service_interface.h>

#include <rw/math/Q.hpp>
#include <rw/common/Timer.hpp>
#include <rw/common/Exception.hpp>
#include <rw/math/MetricUtil.hpp>

#include <rwhw/sdh/SDHDriver.hpp>

#include <ros/ros.h>

#include <string>
#include <sstream>
#include <utility>
#include <cstddef>  // Provides NULL

/* Notes:
 * This node is designed to be run in a single thread that doesn't allow concurrently processing of the set-commands
 * and/or the runLoopHook - eliminating the possibility of race conditions.
 * [ FIXME: invalidated by removal of configureHook() ] The GripperServiceInterface commands (and other services that
 * are configured/advertised in the configureHook()) can be called even when the CarosNode is in a non-running state.
 * This is due to the design choice that all the services and publishers should be broadcasted/available once the
 * CarosNode has been configured. This will avoid hiding (some of) the capabilities/interface of the node when it's not
 * in certain states. However this requires more defensive programming on the interface methods, to make sure that they
 * will only work when the CarosNode has been put in the running state (and connections to the hardware/device has been
 * established or similar).
 */

/* TODO:
 * The node implementation can be greatly simplified if the workaround for the movement of the SDH fingers can be placed
 *in RobWorkHardware or simply removed (requires collecting the debug data and comparing with the SDH temperature).
 * Add support for emergency stop. This can be triggered either through CarosNodeServiceInterface or directly to this
 *node. The SDH library contains a function for emergency stop, which should be used and called immediately.
 *   - Also handle emergency stop properly in the SDHGripperServiceInterface.
 *   - What is required to actually release the emergencystop (within the rwhw::SDHDriver / SDHLibrary)?
 *
 * Allow specification of what interface to use/configure through the parameter server.
 *
 * Could keep track of whether the services/commands (moveQ, gripQ, etc.) finish executing before new commands are
 *received (it's not an error, but maybe it would be nice to get some debug/info information regarding this. Hopefully
 *it would make it easier to reason/debug "what just happened").
 */

SDHNode::SDHNode(const ros::NodeHandle& nodehandle)
    : caros::CarosNodeServiceInterface(nodehandle),
      caros::GripperServiceInterface(nodehandle),
      nodeHandle_(nodehandle),
      currentState_(SDHNode::WAIT),
      nextState_(currentState_),
      sdh_(0)
{
  /* Currently nothing specific should happen */
}

SDHNode::~SDHNode()
{
  if (sdh_ != 0)
  {
    if (sdh_->isConnected())
    {
      ROS_DEBUG_STREAM("Still connected to the SDH device - going to stop the device and disconnect.");
      sdh_->stop();
      sdh_->disconnect();
    }
    delete sdh_;
    sdh_ = 0;
  }
  else
  {
    ROS_DEBUG_STREAM("There was no SDH device to destroy before deallocating/destroying the SDHNode object.");
  }
}

bool SDHNode::activateHook()
{
  if (!configureSDHDevice())
  {
    return false;
  }

  if (!connectToSDHDevice())
  {
    return false;
  }

  return true;
}

bool SDHNode::configureSDHDevice()
{
  if (sdh_ != 0)
  {
    /* Could also just silently return true or false and ignore the error that configure is being invoked twice... */
    CAROS_FATALERROR("The SDH device is already active - trying to configure an already configured SDH node is a bug!",
                     SDHNODE_SDH_DEVICE_ALREADY_ACTIVE);
    return false;
  }
  sdh_ = new rwhw::SDHDriver;

  /* Fetch parameters (if any) or use the defaults */
  nodeHandle_.param("interface_type", interfaceType_, std::string("CAN"));

  nodeHandle_.param("rs232_device", rs232Device_, std::string(""));
  nodeHandle_.param("rs232_port", rs232Port_, 0);
  nodeHandle_.param("rs232_baudrate", rs232BaudRate_, 115200);
  nodeHandle_.param("rs232_timeout", rs232Timeout_, 0.5);

  nodeHandle_.param("can_device", canDevice_, std::string("/dev/pcan0"));
  nodeHandle_.param("can_baudrate", canBaudRate_, 1000000);
  nodeHandle_.param("can_timeout", canTimeout_, 0.5);

  /* TODO: Verify that the chosen interfaceType is valid? or just let it fail when the parameters are being set? */

  /* TODO: Could make the use of the gripper service, configurable through the parameter server. */
  if (not GripperServiceInterface::configureInterface())
  {
    CAROS_FATALERROR("The CAROS GripperServiceInterface could not be configured correctly.",
                     SDHNODE_CAROS_GRIPPER_SERVICE_CONFIGURE_FAIL);
    return false;
  }

  /* Outputting information on supported value ranges */
  /* TODO: This could be made part of the GripperServiceInterface - possibly as a message that is returned (or
   * published) when a client asks for it.
   * If the hardware is intelligent enough to provide new values/boundaries according to position or grasping mode, then
   * it could make sense to publish that information when it changes
   */
  std::pair<rw::math::Q, rw::math::Q> positionLimits = sdh_->getPosLimits();
  rw::math::Q velocityLimits = sdh_->getVelLimits();
  /* There's also getAccLimits() */
  rw::math::Q currentLimits = sdh_->getCurrentLimits();

  ROS_ERROR_STREAM_COND(positionLimits.first.size() != positionLimits.second.size(),
                        "The sizes of the Q's in the position limit pair are not equal; first contains "
                            << positionLimits.first.size() << " and second contains " << positionLimits.second.size()
                            << " elements.");

  ROS_DEBUG_STREAM("Lower position limits: " << positionLimits.first);
  ROS_DEBUG_STREAM("Upper position limits: " << positionLimits.second);
  ROS_DEBUG_STREAM("Velocity limits: " << velocityLimits);
  ROS_DEBUG_STREAM("Current limits: " << currentLimits);

  /* TODO: Debug information on what was configured accoringly to the parameter server? */
  return true;
}

bool SDHNode::connectToSDHDevice()
{
  if (sdh_ == 0)
  {
    CAROS_FATALERROR("The SDH device is not configured", SDHNODE_INTERNAL_ERROR);
    return false;
  }

  if (sdh_->isConnected())
  {
    ROS_ERROR_STREAM(
        "'" << __PRETTY_FUNCTION__
            << "' invoked even though a connection to the SDH device has already been established - this is a bug!");
    return false;
  }

  /* Connect according to interface type and configured parameters */
  if (interfaceType_ == "RS232")
  {
    if (rs232Device_.empty())
    {
      sdh_->connect(rs232Port_, static_cast<unsigned long>(rs232BaudRate_), rs232Timeout_, NULL);
    }
    else
    {
      sdh_->connect(rs232Port_, static_cast<unsigned long>(rs232BaudRate_), rs232Timeout_, rs232Device_.c_str());
    }
  }
  else if (interfaceType_ == "CAN")
  {
    sdh_->connect(canDevice_, canBaudRate_, canTimeout_);
  }
  else
  {
    CAROS_FATALERROR("The specified interface '" << interfaceType_ << "' is not supported.",
                     SDHNODE_UNSUPPORTED_INTERFACE_TYPE);
    return false;
  }

  /* Verify that the connection to the SDH device has been established - this eliminates the need for verifying that the
   * sdh_->connect() function calls actually succeed */
  if (!sdh_->isConnected())
  {
    /* Something went wrong when connecting */
    CAROS_FATALERROR("Failed to properly connect to the SDH device.", SDHNODE_SDH_DEVICE_CONNECT_FAILED);
    return false;
  }

  return true;
}

bool SDHNode::recoverHook()
{
  /* TODO: */
  /* Maybe the connection to the SDH device needs to be reestablished */
  /* Should state be put into the errors or the error system within CarosNodeServiceInterface? (It is not guaranteed
   * that a locally tracked error state is not being superseded by another error cause somewhere else in the CAROS
   * system - so such a solution would be prone to errors) */

  /* Remember to place the state machine in a proper state according to the recovery (e.g. WAIT) */

  ROS_ERROR_STREAM("The recoverHook() has not been implemented yet!");

  return false;
}

void SDHNode::runLoopHook()
{
  try
  {
    if (sdh_ == 0)
    {
      CAROS_FATALERROR("The SDH device is not configured", SDHNODE_INTERNAL_ERROR);
      return;
    }

    if (!sdh_->isConnected())
    {
      CAROS_ERROR("There is no established connection to the SDH device.", SDHNODE_SDH_DEVICE_NO_CONNECTION);
      return;
    }

    /************************************************************************
     * Get Current Joint Positions
     * Currently used as part of the workaround for the movement of the SDH fingers.
    ************************************************************************/
    currentQ_ = sdh_->getQ();

    /************************************************************************
     * Velocity Calculation
     * Used as part of the workaround for the movement of the SDH fingers.
    ************************************************************************/
    double time = velUpdateTimer_.getTime();

    if (lastQ_.size() != currentQ_.size())
    {
      /* lastq_ has not been set before (first time the runloopHook is being called), so set it to currentQ_ */
      lastQ_ = currentQ_;
    }
    else if (time > MIN_TIME_THRESHOLD_FOR_CALCULATING_VELOCITY)
    {
      /* calculate velocity */
      velQ_ = (lastQ_ - currentQ_) / time;
      lastQ_ = currentQ_;
      velUpdateTimer_.resetAndResume();
      ROS_DEBUG_STREAM_NAMED("velocity", "New calculated velocity: " << velQ_);
    }
    ROS_DEBUG_STREAM_NAMED("velocity", "Calculated velocity: " << velQ_);
    ROS_DEBUG_STREAM_NAMED("velocity", "SDH reported velocity: " << sdh_->getdQ());

    /************************************************************************
     * Publish SDH state
     ************************************************************************/
    /* Publishing the SDH state before the state machine because then the measured/sampled values will (probably) have
     * the best match with the current SDH action(s). Reporting a speed and having isstop being true is sort of
     * inconsistent. */
    /* The units of the reported values should match what is specified in the GripperState.msg specification.
     * The rwhw::SDHDriver constructor specifies the use of radians.
     */
    rw::math::Q q = currentQ_;
    /* Using the calculated velocity (FIXME: either continue to use velQ_ or use sdh_->getdQ() if they report similar
     * values - see the velocity debug messages) */
    rw::math::Q dq = velQ_;
    /* FIXME: the current could be converted to a force, given it would make sense to do so - but it requires knowledge
     * of the kinematics to calculate the force that is being applied e.g. at a particular fingertip or where the
     * contact surface is. */
    rw::math::Q force = sdh_->getQCurrent();

    rw::math::Q compare = rw::math::Q::zero(dq.size());
    bool isMoving = (compare != dq) ? true : false;
    bool isBlocked = false;
    /* FIXME: This can possibly give a wrong report in the situation where a new moveQ has just been initiated - so the
     * calculated distance between current position and target is greater than the threshold, but the
     * measured/calculated velocity is still zero */
    /* MBAND DEBUG */
    ROS_INFO_STREAM("mband: currentQ_: " << currentQ_);
    ROS_INFO_STREAM("mband: moveQ_: " << moveQ_);
/* TODO: FIXME:
 * Design flaw, that moveQ_ is 0 (size() == 0) when no move has been initiated... so the distance can be anything
 * depending on initialisation...
 */
/* MBAND END DEBUG */
#if 0
        if (!isMoving && (rw::math::MetricUtil::dist2(currentQ_, moveQ_) >= MOVE_DISTANCE_STOPPED_THRESHOLD)) {
            isBlocked = true;
        }
#endif
    bool isStopped = true;
    /* If not moving nor blocked, then it must be stopped (i.e. reached the target (see GripperState.msg specification))
     */
    if (isMoving || isBlocked)
    {
      isStopped = false;
    }
    /* TODO: properly handle isEmergencyStopped - the logic to register whether an emergency stop is activated or
     * deactivated is missing. Returning false or true in this situation is not recommended though... */
    bool isEmergencyStopped = false;
    publishState(q, dq, force, isMoving, isBlocked, isStopped, isEmergencyStopped);

    /************************************************************************
     * State Machine
     * Used to apply workaround for the movement of the SDH fingers.
    ************************************************************************/
    currentState_ = nextState_;
    switch (currentState_)
    {
      case WAIT:
        /* Do nothing */
        break;
      case MOVE_WAIT:
        /* Workaround to avoid having the SDH try to move the fingers until
         * MAX_TIME_WAITING_FOR_MOVE_TO_FINISH_BEFORE_INTERVENING has passed, when the fingers are almost at their
         * target - (there should be a "bug" causing the SDH to dissipate power when the target can't be reached
         * according to the firmware) */
        if (rw::math::MetricUtil::dist2(currentQ_, moveQ_) < MOVE_DISTANCE_STOPPED_THRESHOLD)
        {
          /* Debug functionality to test the usage of sdh_->waitCmd(0) instead of looking at the remaining distance to
           * the target */
          ROS_DEBUG_STREAM_NAMED("move_wait", "sdh_->waitCmd(0) returned: " << sdh_->waitCmd(0));

          sdh_->stop();
          nextState_ = WAIT;
        }
        else if (moveStartTimer_.getTime() > MAX_TIME_WAITING_FOR_MOVE_TO_FINISH_BEFORE_INTERVENING)
        {
          ROS_DEBUG_STREAM_NAMED("move_wait", "Waited long enough to possible intervene.");
          if (rw::math::MetricUtil::normInf(velQ_) < MINIMUM_VELOCITY_BEFORE_CONSIDERED_NOT_MOVING)
          {
            ROS_DEBUG_STREAM_NAMED("move_wait", "Intervening due to the fingers not being considered moving.");
            sdh_->stop();
            nextState_ = WAIT;
          }
        }
        break;
      default:
        ROS_FATAL_STREAM("Unknown state in the SDH state machine '"
                         << currentState_ << "' (a value is expected due to enum implementation) - This is a bug!");
        /* This is considered a fatal error, but should never happen. */
        CAROS_FATALERROR("Unknown state in the SDH state machine", SDHNODE_INTERNAL_ERROR);
        break;
    }
  }
  catch (const rw::common::Exception& exp)
  {
    /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw
     * more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier
     * to do error recovery. */
    CAROS_ERROR(exp.what(), SDHNODE_INTERNAL_ERROR);
    return;
  }
}

void SDHNode::errorLoopHook()
{
  /* Stop the SDH's current action(s) */
  if (sdh_ == 0)
  {
    ROS_DEBUG_STREAM("The SDH device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
  }
  else
  {
    if (sdh_->isConnected())
    {
      sdh_->stop();
    }
  }
}

void SDHNode::fatalErrorLoopHook()
{
  /* Stop the SDH's current action(s) */
  if (sdh_ == 0)
  {
    ROS_DEBUG_STREAM("The SDH device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
  }
  else
  {
    if (sdh_->isConnected())
    {
      sdh_->stop();
      /* A fatal error should disconnect the SDH device */
      sdh_->disconnect();
    }
  }
}

/************************************************************************
 * GripperServiceInterface
 ************************************************************************/
/* Note:
 * The checks isInRunning(), (sdh_ == 0) and (! sdh_->isConnected()) are not placed in one common function, because the
 * CAROS_ERROR and CAROS_FATALERROR macros are using source code lines to sort of pinpoint the "faulty" function.
 * When a more appropriate method is found that can reduce this code duplication, then it should be implemented! (A
 * preprocessor code generating macro is not exactly a nice and easily maintainable solution)
 */
bool SDHNode::moveQ(const rw::math::Q& q)
{
  ROS_DEBUG_STREAM_NAMED("received_q", "moveQ: " << q);

  if (!isInRunning())
  {
    ROS_WARN_STREAM("Not in running state!");
    return false;
  }

  if (sdh_ == 0)
  {
    CAROS_FATALERROR("The SDH device is not configured.", SDHNODE_NO_SDH_DEVICE);
    return false;
  }

  if (!sdh_->isConnected())
  {
    CAROS_ERROR("There is no established connection to the SDH device.", SDHNODE_SDH_DEVICE_NO_CONNECTION);
    return false;
  }

  if (q.size() != SUPPORTED_Q_LENGTH_FOR_SDHNODE)
  {
    CAROS_ERROR("The length of Q is " << q.size() << " but should be " << SUPPORTED_Q_LENGTH_FOR_SDHNODE,
                SDHNODE_UNSUPPORTED_Q_LENGTH);
    return false;
  }

  try
  {
    moveQ_ = q;
    moveStartTimer_.resetAndResume();

    sdh_->moveCmd(moveQ_);
    nextState_ = MOVE_WAIT;
  }
  catch (const rw::common::Exception& exp)
  {
    /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw
     * more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier
     * to do error recovery. */
    CAROS_ERROR(exp.what(), SDHNODE_INTERNAL_ERROR);
    return false;
  }

  return true;
}

bool SDHNode::gripQ(const rw::math::Q& q)
{
  ROS_DEBUG_STREAM_NAMED("received_q", "gripQ: " << q);

  if (!isInRunning())
  {
    ROS_WARN_STREAM("Not in running state!");
    return false;
  }

  if (sdh_ == 0)
  {
    CAROS_FATALERROR("The SDH device is not configured.", SDHNODE_NO_SDH_DEVICE);
    return false;
  }

  if (!sdh_->isConnected())
  {
    CAROS_ERROR("There is no established connection to the SDH device.", SDHNODE_SDH_DEVICE_NO_CONNECTION);
    return false;
  }

  if (q.size() != SUPPORTED_Q_LENGTH_FOR_SDHNODE)
  {
    CAROS_ERROR("The length of Q is " << q.size() << " but should be " << SUPPORTED_Q_LENGTH_FOR_SDHNODE,
                SDHNODE_UNSUPPORTED_Q_LENGTH);
    return false;
  }

  try
  {
    sdh_->moveCmd(q);
    /* Do nothing; letting the SDH continue to apply force as part of its grasp */
    nextState_ = WAIT;
  }
  catch (const rw::common::Exception& exp)
  {
    /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw
     * more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier
     * to do error recovery. */
    CAROS_ERROR(exp.what(), SDHNODE_INTERNAL_ERROR);
    return false;
  }

  return true;
}

bool SDHNode::setForceQ(const rw::math::Q& q)
{
  ROS_DEBUG_STREAM_NAMED("received_q", "setForceQ: " << q);

  if (!isInRunning())
  {
    ROS_WARN_STREAM("Not in running state!");
    return false;
  }

  if (sdh_ == 0)
  {
    CAROS_FATALERROR("The SDH device is not configured.", SDHNODE_NO_SDH_DEVICE);
    return false;
  }

  if (!sdh_->isConnected())
  {
    CAROS_ERROR("There is no established connection to the SDH device.", SDHNODE_SDH_DEVICE_NO_CONNECTION);
    return false;
  }

  if (q.size() != SUPPORTED_Q_LENGTH_FOR_SDHNODE)
  {
    CAROS_ERROR("The length of Q is " << q.size() << " but should be " << SUPPORTED_Q_LENGTH_FOR_SDHNODE,
                SDHNODE_UNSUPPORTED_Q_LENGTH);
    return false;
  }

  try
  {
    sdh_->setTargetQCurrent(q);
  }
  catch (const rw::common::Exception& exp)
  {
    /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw
     * more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier
     * to do error recovery. */
    CAROS_ERROR(exp.what(), SDHNODE_INTERNAL_ERROR);
    return false;
  }

  return true;
}

bool SDHNode::setVelocityQ(const rw::math::Q& q)
{
  ROS_DEBUG_STREAM_NAMED("received_q", "setVelocityQ: " << q);

  if (!isInRunning())
  {
    ROS_WARN_STREAM("Not in running state!");
    return false;
  }

  if (sdh_ == 0)
  {
    CAROS_FATALERROR("The SDH device is not configured.", SDHNODE_NO_SDH_DEVICE);
    return false;
  }

  if (!sdh_->isConnected())
  {
    CAROS_ERROR("There is no established connection to the SDH device.", SDHNODE_SDH_DEVICE_NO_CONNECTION);
    return false;
  }

  if (q.size() != SUPPORTED_Q_LENGTH_FOR_SDHNODE)
  {
    CAROS_ERROR("The length of Q is " << q.size() << " but should be " << SUPPORTED_Q_LENGTH_FOR_SDHNODE,
                SDHNODE_UNSUPPORTED_Q_LENGTH);
    return false;
  }

  try
  {
    sdh_->setTargetQVel(q);
  }
  catch (const rw::common::Exception& exp)
  {
    /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw
     * more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier
     * to do error recovery. */
    CAROS_ERROR(exp.what(), SDHNODE_INTERNAL_ERROR);
    return false;
  }

  return true;
}

bool SDHNode::stopMovement()
{
  if (!isInRunning())
  {
    ROS_WARN_STREAM("Not in running state!");
    return false;
  }

  if (sdh_ == 0)
  {
    CAROS_FATALERROR("The SDH device is not configured.", SDHNODE_NO_SDH_DEVICE);
    return false;
  }

  if (!sdh_->isConnected())
  {
    CAROS_ERROR("There is no established connection to the SDH device.", SDHNODE_SDH_DEVICE_NO_CONNECTION);
    return false;
  }

  try
  {
    sdh_->stop();
    nextState_ = WAIT;
  }
  catch (const rw::common::Exception& exp)
  {
    /* TODO: Digest the exp.what() string and find more appropriate error codes - or improve rwhw::SDHDriver to throw
     * more c++ standard exceptions with out-of-range and invalid parameter specific exceptions. Thus making it easier
     * to do error recovery. */
    CAROS_ERROR(exp.what(), SDHNODE_INTERNAL_ERROR);
    return false;
  }

  return true;
}
