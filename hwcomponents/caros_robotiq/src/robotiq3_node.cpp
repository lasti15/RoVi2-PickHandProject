#include <caros/robotiq3_node.h>
#include <rw/rw.hpp>

USE_ROBWORK_NAMESPACE

using namespace caros;
using namespace robwork;

Robotiq3Node::Robotiq3Node(const ros::NodeHandle& node_handle)
    : caros::CarosNodeServiceInterface(node_handle),
      caros::GripperServiceInterface(node_handle),
      last_Q_(4, 0, 0, 0, 0),
      robotiq3_(NULL),
      node_handle_(node_handle)

{
  /* Currently nothing specific should happen */
}

Robotiq3Node::~Robotiq3Node()
{
  if (robotiq3_ != NULL)
  {
    if (robotiq3_->isConnected())
    {
      ROS_DEBUG_STREAM("Still connected to the Robotiq3 device - going to stop the device and disconnect.");
      robotiq3_->disconnect();
    }
    robotiq3_ = NULL;
  }
  else
  {
    ROS_DEBUG_STREAM("There was no Robotiq3 device to destroy before deallocating/destroying the Robotci3Node object.");
  }
}

bool Robotiq3Node::activateHook()
{
  if (!configureRobotiqDevice())
  {
    return false;
  }

  if (!connectToRobotiqDevice())
  {
    return false;
  }

  return true;
}

bool Robotiq3Node::recoverHook(const std::string& errorMsg, const int64_t errorCode)
{
  /* TODO: */

  ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " has not been implemented yet!");
  ROS_BREAK();

  return false;
}

void Robotiq3Node::runLoopHook()
{
  try
  {
    if (robotiq3_ == 0)
    {
      CAROS_FATALERROR("The Robotiq3 device is not configured", ROBOTIQ3NODE_INTERNAL_ERROR);
      return;
    }

    if (!robotiq3_->isConnected())
    {
      CAROS_ERROR("There is no established connection to the Robotiq3 device.",
                  ROBOTIQ3NODE_ROBOTIQ_DEVICE_NO_CONNECTION);
      return;
    }

    /************************************************************************
     * Get the time since last time this function was run.
     ************************************************************************/
    ros::Time now = ros::Time::now();
    ros::Duration diff = now - last_loop_time_;

    /************************************************************************
     * Get current gripper state and split values
     ************************************************************************/
    robotiq3_->getAllStatusCMD();
    Q q = robotiq3_->getQ();
    Q dq_calc = (q - last_Q_) / diff.toSec();
    Q force = robotiq3_->getQCurrent();
    bool is_moving = robotiq3_->isGripperMoving();
    bool is_blocked = robotiq3_->isGripperBlocked();
    bool is_stopped = !robotiq3_->isGripperMoving() && !robotiq3_->isGripperBlocked();
    /* FIXME: hardcoded isEstop value */
    bool is_emergency_stopped = false;
    publishState(q, dq_calc, force, is_moving, is_blocked, is_stopped, is_emergency_stopped);

    last_Q_ = q;
    last_loop_time_ = now;
  }
  catch (const rw::common::Exception& exp)
  {
    CAROS_ERROR(exp.what(), ROBOTIQ3NODE_INTERNAL_ERROR);
    return;
  }
}

void Robotiq3Node::errorLoopHook()
{
  /* Stop the Robotiq's current action(s) */
  if (robotiq3_ == 0)
  {
    ROS_DEBUG_STREAM("The Robotiq3 device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
  }
  else
  {
    robotiq3_->stopCmd();
    robotiq3_->disconnect();
  }
}

void Robotiq3Node::fatalErrorLoopHook()
{
  /* Stop the Robotiq's current action(s) */
  if (robotiq3_ == 0)
  {
    ROS_DEBUG_STREAM("The Robotiq3 device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
  }
  else
  {
    robotiq3_->stopCmd();
    robotiq3_->disconnect();
  }
}

bool Robotiq3Node::configureRobotiqDevice()
{
  if (robotiq3_ != 0)
  {
    /* Could also just silently return true or false and ignore the error that configure is being invoked twice... */
    CAROS_FATALERROR(
        "The Robotiq3 device is already active - trying to configure an already configured Robotiq3 node is a bug!",
        ROBOTIQ3NODE_ROBOTIQ_DEVICE_ALREADY_ACTIVE);
    return false;
  }

  /* Fetch parameters (if any) or use the defaults */
  node_handle_.param("ip", ip_, std::string("192.168.100.21"));
  node_handle_.param("port", port_, 502);

  // TODO: Verify that the chosen parameters are valid?

  robotiq3_ = ownedPtr(new rwhw::Robotiq3());

  if (not GripperServiceInterface::configureInterface())
  {
    CAROS_FATALERROR("The CAROS GripperService could not be configured correctly.",
                     ROBOTIQ3NODE_CAROS_GRIPPER_SERVICE_CONFIGURE_FAIL);
    return false;
  }

  /* Outputting information on supported value ranges */
  typedef std::pair<rw::math::Q, rw::math::Q> pair_q;
  pair_q position_limits = robotiq3_->getLimitPos();
  pair_q velocity_limits = robotiq3_->getLimitVel();
  pair_q force_limits = robotiq3_->getLimitForce();

  ROS_ERROR_STREAM_COND(position_limits.first.size() != position_limits.second.size(),
                        "The sizes of the Q's in the position limit pair are not equal. first contains "
                            << position_limits.first.size() << " and second contains " << position_limits.second.size()
                            << " elements.");
  ROS_ERROR_STREAM_COND(velocity_limits.first.size() != velocity_limits.second.size(),
                        "The sizes of the Q's in the velocity limit pair are not equal. first contains "
                            << velocity_limits.first.size() << " and second contains " << velocity_limits.second.size()
                            << " elements.");
  ROS_ERROR_STREAM_COND(force_limits.first.size() != force_limits.second.size(),
                        "The sizes of the Q's in the force limit pair are not equal. first contains "
                            << force_limits.first.size() << " and second contains " << force_limits.second.size()
                            << " elements.");
  ROS_DEBUG_STREAM("Lower position limits: " << position_limits.first);
  ROS_DEBUG_STREAM("Upper position limits: " << position_limits.second);
  ROS_DEBUG_STREAM("Lower velocity limits: " << velocity_limits.first);
  ROS_DEBUG_STREAM("Upper velocity limits: " << velocity_limits.second);
  ROS_DEBUG_STREAM("Lower force limits: " << force_limits.first);
  ROS_DEBUG_STREAM("Upper force limits: " << force_limits.second);

  /* TODO: Debug information on what was configured accordingly to the parameter server? */
  return true;
}

bool Robotiq3Node::connectToRobotiqDevice()
{
  if (robotiq3_ == 0)
  {
    CAROS_FATALERROR("The Robotiq3 device is not configured", ROBOTIQ3NODE_INTERNAL_ERROR);
    return false;
  }

  if (robotiq3_->isConnected())
  {
    ROS_ERROR_STREAM(
        "'"
        << __PRETTY_FUNCTION__
        << "' invoked even though a connection to the Robotiq3 device has already been established - this is a bug!");
    return false;
  }

  /* Connect according to configured parameters */
  if (!robotiq3_->connect(ip_, port_))
  {
    CAROS_FATALERROR("The Robotiq3 hand was not able to connect to" << ip_ << " : " << port_, CONNECTION_ERROR);
    return false;
  }

  /* Verify that the connection to the Robotiq3 device has been established - this eliminates the need for verifying
   * that the _robotiq->connect() function calls actually succeed */
  if (!robotiq3_->isConnected())
  {
    /* Something went wrong when connecting */
    CAROS_FATALERROR("Failed to properly connect to the Robotiq3 device.", ROBOTIQ3NODE_ROBOTIQ_DEVICE_CONNECT_FAILED);
    return false;
  }

  return true;
}

/************************************************************************
 * GripperServiceInterface
 ************************************************************************/
bool Robotiq3Node::moveQ(const rw::math::Q& q)
{
  if (not isInWorkingCondition())
  {
    return false;
  }

  ROS_DEBUG_STREAM("moveQ with " << q.size() << " joint(s).");

  try
  {
    robotiq3_->moveCmd(q);
    last_cmd_ = MOVE;
  }
  catch (const rw::common::Exception& exp)
  {
    CAROS_ERROR(exp.what(), ROBOTIQ3NODE_INTERNAL_ERROR);
    return false;
  }
  return true;
}

bool Robotiq3Node::gripQ(const rw::math::Q& q)
{
  if (not isInWorkingCondition())
  {
    return false;
  }

  try
  {
    robotiq3_->moveCmd(q);
    last_cmd_ = GRIP;
  }
  catch (const rw::common::Exception& exp)
  {
    CAROS_ERROR(exp.what(), ROBOTIQ3NODE_INTERNAL_ERROR);
    return false;
  }
  return true;
}

bool Robotiq3Node::setForceQ(const rw::math::Q& q)
{
  if (not isInWorkingCondition())
  {
    return false;
  }

  try
  {
    robotiq3_->setTargetQForce(q);
  }
  catch (const rw::common::Exception& exp)
  {
    CAROS_ERROR(exp.what(), ROBOTIQ3NODE_INTERNAL_ERROR);
    return false;
  }
  return true;
}

bool Robotiq3Node::setVelocityQ(const rw::math::Q& q)
{
  if (not isInWorkingCondition())
  {
    return false;
  }

  try
  {
    robotiq3_->setTargetQVel(q);
  }
  catch (const rw::common::Exception& exp)
  {
    CAROS_ERROR(exp.what(), ROBOTIQ3NODE_INTERNAL_ERROR);
    return false;
  }

  return true;
}

bool Robotiq3Node::stopMovement()
{
  if (not isInWorkingCondition())
  {
    return false;
  }

  try
  {
    robotiq3_->stopCmd();
  }
  catch (const rw::common::Exception& exp)
  {
    CAROS_ERROR(exp.what(), ROBOTIQ3NODE_INTERNAL_ERROR);
    return false;
  }

  return true;
}

/************************************************************************
 * Utility functions
 ************************************************************************/
bool Robotiq3Node::isInWorkingCondition()
{
  if (!isInRunning())
  {
    ROS_WARN_STREAM("Not in running state!");
    return false;
  }

  if (robotiq3_ == 0)
  {
    CAROS_FATALERROR("The Robotiq3 device is not configured.", ROBOTIQ3NODE_NO_ROBOTIQ_DEVICE);
    return false;
  }

  if (not robotiq3_->isConnected())
  {
    CAROS_ERROR("There is no established connection to the Robotiq3 device.",
                ROBOTIQ3NODE_ROBOTIQ_DEVICE_NO_CONNECTION);
    return false;
  }

  return true;
}
