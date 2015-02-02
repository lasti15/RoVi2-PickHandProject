#ifndef CAROS_CAROSNODESERVICEINTERFACE_HPP_
#define CAROS_CAROSNODESERVICEINTERFACE_HPP_

#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include <string>

#define CAROS_NODE_SERVICE_INTERFACE_SUB_NAMESPACE "caros_node"

/************************************************************************
 * TODO:
 * - Add apidoc documentation
 * - Error codes: Make use of a "singleton counter" and the parameter
 *                server to handle the distribution of error codes and
 *                their corresponding human-friendly description.
 * - [IMPORTANT] Update the documentation according to the new simplified
 *   statemachine.
 ************************************************************************/

/**
 * @brief Emit an CAROS node error. It will also be emitted to ROS_ERROR
 *
 * \b ostreamExpression is an expression that is fed to an output stream. Example:
\code
CAROS_ERROR("The value of x is " << x << ". x should be less than zero.", 2);
\endcode
 *
 */
#define CAROS_ERROR(ostreamExpression, errorCode)                                                 \
  do                                                                                              \
  {                                                                                               \
    std::stringstream ERROR__stream;                                                              \
    ERROR__stream << ostreamExpression;                                                           \
    ROS_ERROR_STREAM("CarosNodeError: " << ERROR__stream.str() << "; error code: " << errorCode); \
    error(ERROR__stream.str(), errorCode);                                                        \
  } while (0)

/**
 * @brief Emit an CAROS node fatal error. It will also be emitted to ROS_ERROR
 *
 * \b ostreamExpression is an expression that is fed to an output stream. Example:
\code
CAROS_FATALERROR("The value of x is " << x << ". x must not be less than zero.", 5);
\endcode
 *
 */
#define CAROS_FATALERROR(ostreamExpression, errorCode)                                                      \
  do                                                                                                        \
  {                                                                                                         \
    std::stringstream FATALERROR__stream;                                                                   \
    FATALERROR__stream << ostreamExpression;                                                                \
    ROS_ERROR_STREAM("CarosNodeFatalError: " << FATALERROR__stream.str() << "; error code: " << errorCode); \
    fatalError(FATALERROR__stream.str(), errorCode);                                                        \
  } while (0)

namespace caros
{
enum CAROS_NODE_ERRORCODE
{
  CAROS_NODE_NO_ERROR_CODE_SUPPLIED = 0
};

/* FIXME: the description below is now outdated */
/**
 * @brief A node service interface that defines a simple statemachine from which
 * the node can be controlled.
 *
 * [ FIXME This information is outdated FIXME ]
 *
 * There are 5 states: init, stopped, running, error, fatalerror
 *
 * The following transitions are legal
 * init: (configure)
 * ->fatalerror (through configure)
 * ->stopped (through configure)
 * stopped: (cleanup, start, error, ferror)
 * ->init (through cleanup)
 * ->running (through start)
 * ->error( through error)
 * ->fatalerror(through ferror)
 * running: (stop,error,ferror)
 * ->stopped (through stop)
 * ->error (through error)
 * ->fatalerror (through ferror)
 * error: (cleanup, retry)
 * ->running(retry)
 * ->init(through cleanup)
 * ->stopped(through retry)
 *
 *
 *
 *
 */
class CarosNodeServiceInterface
{
 public:
  /**
   * @brief constructor.
   * @param[in] nodehandle The nodehandle to use for ROS services and publishers.
   * @param[in] loopRateFrequency Optional parameter that specifies the frequency [Hz] of this ROS node - see
   * setLoopRateFrequency() for more information.
   */
  CarosNodeServiceInterface(const ros::NodeHandle& nodehandle, const double loopRateFrequency = 30);

  /**
   * @brief virtual destructor
   */
  virtual ~CarosNodeServiceInterface();

  /**
   * @brief Start the CAROS node
   *
   * Invoke this function to hand over the control to the CAROS node statemachine.
   * This is a blocking function that will return when ROS is being told to shutdown.
   */
  void start();

  /**
   * @brief The states that the CAROS node can be in
   */
  enum NodeState
  {
    PREINIT = 0,
    RUNNING,
    INERROR,
    INFATALERROR
  };

 protected:
  /* TODO:
   * Should the Hook description mention something about deriving/child classes?
   * These apidocs contains a lot of duplicated information - should that be deduplicated or is there a better
   * alternative?
   */
  /**
   * @name ROS Service Hooks
   * @brief These hooks needs to be implemented in the deriving node, allowing for a common interface for controlling
   * CAROS nodes.
   */
  /** @{ */
  /**
   * @brief This is called when the node is transitioning to the RUNNING state, which will happen automatically when
   *invoking start()
   *
   * This hook should be used to establish connections to the hardware and activate the hardware.
   * It is also here that other interfaces should be initialised (e.g. the CAROS GripperServiceInterface), together with
   *advertising ROS services and publishers that are specific to the node.
   * If an error occurs, then either error() or fatalError() should be called depending on the severity of the error,
   *and false returned.
   */
  virtual bool activateHook() = 0;

  /**
   * @brief This is called as part of a recovery process that is invoked through the ROS service "recover".
   *
   * This hook should be used to perform any necessary steps to recover from the error.
   * The design of the recovery process is to be considered incomplete. Some things are missing such as the ability to
   *properly see what error the node has been told to recover from (it's available in the CarosNodeService, but how to
   *properly use it when recovering is undecided).
   */
  virtual bool recoverHook() = 0;
  /** @} */

  /**
   * @name Loop Hooks
   *
   * The loop hook corresponding to the current state will be invoked at the frequency specified when calling the
   *constructor CarosNodeServiceInterface(const ros::NodeHandle& nodehandle, const double loopRateFrequency) or set
   *through setLoopRateFrequency().
   */
  /** @{ */
  virtual void runLoopHook() = 0;
  virtual void errorLoopHook() {/* Empty */};
  virtual void fatalErrorLoopHook() {/* Empty */};
  /** @} */

  void error(const std::string& msg, const int64_t errorCode = CAROS_NODE_NO_ERROR_CODE_SUPPLIED);
  void fatalError(const std::string& msg, const int64_t errorCode = CAROS_NODE_NO_ERROR_CODE_SUPPLIED);

  NodeState getState()
  {
    return _nodeState;
  }
  NodeState getPreviousState()
  {
    return _previousState;
  }

  /** @{ */
  bool isInRunning()
  {
    return _nodeState == RUNNING;
  }
  bool isInError()
  {
    return _nodeState == INERROR;
  }
  bool isInFatalError()
  {
    return _nodeState == INFATALERROR;
  }
  /** @} */

  /**
   * @brief Change the frequency of this ROS node.
   *
   * @param[in] frequency The new frequency [Hz].
   *
   * Change how often this node is supposed to execute ROS service callbacks and publish CAROS node messages.
   * A very small value or a negative value will (according to the current roscpp implementation) cause this ROS node to
   *process the service callbacks and publishers as fast as possible.
   */
  void setLoopRateFrequency(const double frequency);

  /**
   * @brief Get the frequency of this ROS node.
   *
   * @returns The frequency [Hz].
   */
  double getLoopRateFrequency()
  {
    return _loopRateFrequency;
  }

 private:
  /**
   * @brief private default constructor.
   * This is declared as private to enforce deriving classes to call an available public constructor and enforce that
   * the ROS services are properly initialised.
   */
  CarosNodeServiceInterface();

  /** @{ */
  /**
   * @brief Activate the CAROS node.
   *
   * This transitions the node into the RUNNING state. This should have the node establish connections to the hardware
   *and activate it, together with advertising the ROS services and publishers for the node.
   *
   * @pre In PREINIT state
   * @post Success: in RUNNING state
   * @post Failure: in one of the error states, depending on the severity of the failure
   */
  bool activateNode();

  /**
   * @brief Recover from an error.
   *
   * If it's possible to recover then the node will transition back into the state that the node was in before entering
   *the INERROR state.
   *
   * @pre In INERROR state
   * @post Success: in the previous state
   * @post Failure: In INERROR state [ TODO: Settle on a strategy possibly involving INFATALERROR ]
   */
  bool recoverNode();
  /** @} */

  /**
   * @brief internal GripperServiceInterface function to properly advertise the services using ROS and setup the
   *callbacks.
   *
   * This should not be called directly.
   */
  bool initCarosNode();

  /** @{ */
  /**
   * @brief ROS service wrapper for recoverNode().
   */
  bool recoverHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  /** @} */

  void changeState(const NodeState newState);

  void publishNodeState(const bool stateChanged = false);

 private:
  ros::NodeHandle _nodeHandle;

  ros::Publisher _nodeStatePublisher;

  ros::ServiceServer _srvRecover;

  NodeState _nodeState;
  NodeState _previousState;

  /* TODO:
   * Should the _loopRateFrequency be settable through the CarosNodeServiceInterface that is exposed as ROS services?
   */
  double _loopRateFrequency;
  ros::Rate _loopRate;

  std::string _errorMsg;
  /* Using int64_t because it's highly related with the type specified in the caros_common_msgs::CarosNodeState message
   */
  int64_t _errorCode;
};
}  // namespace
#endif
