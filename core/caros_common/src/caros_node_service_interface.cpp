#include <caros/caros_node_service_interface.h>
#include <caros_common_msgs/caros_node_state.h>

#include <ros/ros.h>

#include <string>

namespace caros
{
namespace
{
/* The order of the states _has_ to be the same as the order defined in CarosNodeServiceInterface.hpp for NodeState */
static std::string CarosStateString[] = {"PREINIT", "RUNNING", "ERROR", "FATALERROR"};
}

CarosNodeServiceInterface::CarosNodeServiceInterface(const ros::NodeHandle& nodehandle, const double loopRateFrequency)
    : nodeHandle_(nodehandle, CAROS_NODE_SERVICE_INTERFACE_SUB_NAMESPACE),
      nodeState_(PREINIT),
      loopRateFrequency_(loopRateFrequency),
      loopRate_(loopRateFrequency_)
{
  if (not initCarosNode())
  {
    throw std::runtime_error("Could not properly initialise the required basic services and/or publishers.");
  }
}

CarosNodeServiceInterface::~CarosNodeServiceInterface()
{
  /* Nothing specific to destroy */
}

void CarosNodeServiceInterface::start()
{
  if (!activateNode())
  {
    ROS_DEBUG_STREAM(
        "activateNode() was unsuccessful - the node should now be in an error state according to best practices.");
  }

  caros_common_msgs::caros_node_state state;
  while (ros::ok())
  {
    ros::spinOnce();

    ROS_FATAL_STREAM_COND(nodeState_ == PREINIT, "The CAROS node is not supposed to be in the state '"
                                                     << CarosStateString[PREINIT]
                                                     << "' at this point - this is a bug!");
    ROS_ASSERT(nodeState_ != PREINIT);

    /* Transitions to the error states will be handled right away in the same ROS cycle */

    if (nodeState_ == RUNNING)
    {
      runLoopHook();
    }
    /* Process errors if any occurred */
    if (nodeState_ == ERROR)
    {
      errorLoopHook();
    }
    /* Also process fatal error state (if an error becomes a fatal error, then it will also be processed in the same ROS
     * cycle) */
    if (nodeState_ == FATALERROR)
    {
      fatalErrorLoopHook();
    }

    publishNodeState();

    /* Sleep in order to run approximately at the specified frequency */
    loopRate_.sleep();
    /* TODO:
     * Replace this sleeping using a ros::Rate with a ros::Timer using a callback function - that is the recommended way
     * to do it according to http://wiki.ros.org/roscpp/Overview/Time#Sleeping_and_Rates
     * Also output some debug information related to how well the scheduling is going, so delays can easily be seen in
     * the logs.
     */
  }
}

bool CarosNodeServiceInterface::activateNode()
{
  // can only be called when in PREINIT state
  if (nodeState_ != PREINIT)
  {
    ROS_ERROR_STREAM("Activate can only be called when in " << CarosStateString[PREINIT] << " state. The node was in "
                                                            << CarosStateString[nodeState_] << ".");
    return false;
  }

  if (activateHook())
  {
    changeState(RUNNING);
  }
  else
  {
    /* It is considered a fatal error if the activateHook() failed and this object was not placed in an error state! */
    NodeState currentState = getState();
    if (currentState != ERROR && currentState != FATALERROR)
    {
      ROS_FATAL_STREAM("activateHook() failed and the CAROS node was never put into an error state - this is a bug!");
    }
    else
    {
      ROS_DEBUG_STREAM("activateHook() failed.");
    }

    return false;
  }

  return true;
}

bool CarosNodeServiceInterface::recoverNode()
{
  // can only be called when in ERROR state
  if (nodeState_ != ERROR)
  {
    ROS_WARN_STREAM("Recover can only be called from " << CarosStateString[ERROR] << " state. The node was in "
                                                       << CarosStateString[nodeState_] << ".");
    return false;
  }

  if (recoverHook())
  {
    ROS_ERROR_STREAM_COND(previousState_ == FATALERROR, "A successful recovery brings the node back into the "
                                                            << CarosStateString[FATALERROR]
                                                            << " state - This is a bug!");
    changeState(previousState_);
  }
  else
  {
    ROS_DEBUG_STREAM("recoverHook() failed.");

    return false;
  }

  return true;
}

bool CarosNodeServiceInterface::terminateNode()
{
  /* Can be called at all times */
  /* Simply request shutdown - a non blocking call and shutdown will occur at next "ROS cycle". */
  ros::requestShutdown();

  return true;
}

void CarosNodeServiceInterface::error(const std::string& msg, const int64_t errorCode)
{
  ROS_DEBUG_STREAM("CarosNodeError: " << msg << "; error code: " << errorCode);
  /* keep a copy of the error message so it can be published */
  errorMsg_ = msg;
  errorCode_ = errorCode;
  changeState(ERROR);
}

void CarosNodeServiceInterface::fatalError(const std::string& msg, const int64_t errorCode)
{
  ROS_DEBUG_STREAM("CarosNodeFatalError: " << msg << "; error code: " << errorCode);
  /* keep a copy of the (fatal) error message so it can be published */
  errorMsg_ = msg;
  errorCode_ = errorCode;
  changeState(FATALERROR);
}

void CarosNodeServiceInterface::setLoopRateFrequency(const double frequency)
{
  ROS_DEBUG_STREAM("Changing the loop rate frequency from " << loopRateFrequency_ << " to " << frequency);
  loopRateFrequency_ = frequency;
  loopRate_ = ros::Rate(loopRateFrequency_);
}

void CarosNodeServiceInterface::changeState(const NodeState newState)
{
  ROS_DEBUG_STREAM("Changing state from " << CarosStateString[nodeState_] << " to " << CarosStateString[newState]);
  if (newState != nodeState_)
  {
    previousState_ = nodeState_;
    nodeState_ = newState;
    publishNodeState(true);
  }
  else
  {
    ROS_DEBUG_STREAM("Not changing state as the new state is the same as the current state!");
  }
}

bool CarosNodeServiceInterface::initCarosNode()
{
  if (nodeStatePublisher_ || srvRecover_ || srvTerminate_)
  {
    ROS_WARN_STREAM(
        "Reinitialising one or more CarosNodeServiceInterface services or publishers. If this is not fully intended "
        "then this should be considered a bug!");
  }

  nodeStatePublisher_ = nodeHandle_.advertise<caros_common_msgs::caros_node_state>("caros_node_state", 1);
  ROS_ERROR_STREAM_COND(!nodeStatePublisher_, "The caros_node_state publisher is empty!");

  srvRecover_ = nodeHandle_.advertiseService("recover", &CarosNodeServiceInterface::recoverHandle, this);
  ROS_ERROR_STREAM_COND(!srvRecover_, "The recover service is empty!");

  srvTerminate_ = nodeHandle_.advertiseService("terminate", &CarosNodeServiceInterface::terminateHandle, this);
  ROS_ERROR_STREAM_COND(!srvTerminate_, "The terminate service is empty!");

  if (nodeStatePublisher_ && srvRecover_ && srvTerminate_)
  {
    /* Everything seems to be properly initialised */
  }
  else
  {
    ROS_FATAL_STREAM("One or more of the ROS publishers or services could not be properly initialised.");
    return false;
  }

  return true;
}

bool CarosNodeServiceInterface::recoverHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  return recoverNode();
}

bool CarosNodeServiceInterface::terminateHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  return terminateNode();
}

void CarosNodeServiceInterface::publishNodeState(const bool stateChanged)
{
  caros_common_msgs::caros_node_state state;
  state.state = CarosStateString[nodeState_];
  state.inError = nodeState_ == ERROR || nodeState_ == FATALERROR;
  if (state.inError)
  {
    state.errorMsg = errorMsg_;
    state.errorCode = errorCode_;
  }
  /* The errorMsg is not being cleared when the state no longer is in error, this is intended to provide a minor error
   * log / history of errors.
   * TODO: Provide a history/log of the last N errors together with some info such as a timestamp (maybe how long the
   * node was in the error state) and similar.
   */

  state.changedEvent = stateChanged;

  nodeStatePublisher_.publish(state);
}
}  // namespace caros
