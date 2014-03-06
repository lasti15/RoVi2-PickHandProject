#ifndef CAROS_CAROSNODESERVICEINTERFACE_HPP_
#define CAROS_CAROSNODESERVICEINTERFACE_HPP_

#include <rw/math/Q.hpp>
#include <rw/common/Ptr.hpp>

#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include <string>



namespace caros {
  /**
   * @brief A node service interface that defines a simple statemachine from which
   * the node can be controlled.
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
  class CarosNodeServiceInterface {
  public:

    /**
     * @brief constructor.
     * @param nodehandle [in] the nodehandle to use for services.
     */
    CarosNodeServiceInterface(rw::common::Ptr<ros::NodeHandle> nodehandle);

    /**
     * @brief constructor
     * @param service_name [in] the namespace used for creating a nodehandle.
     */
    CarosNodeServiceInterface(const std::string& service_name);

    /**
     * @brief virtual destructor
     */
    virtual ~CarosNodeServiceInterface();

    void start();

    bool startNode();
    bool stopNode();
    bool configureNode();
    bool cleanupNode();
    bool recoverNode();


    std::string getErrorMsg();

    // possible node states
    typedef enum {PREINIT,STOPPED,RUNNING,INERROR,INFATALERROR} NodeState;

  protected:

    virtual void runloopHook() = 0;
    virtual void configureHook() = 0;
    virtual void cleanupHook() = 0;
    virtual void startHook() = 0;
    virtual void stopHook() = 0;
    virtual void recoverHook() = 0;


    virtual void stoppedLoopHook(){};
    virtual void initLoopHook(){};
    virtual void errorLoopHook(){};
    virtual void fatalerrorLoopHook(){};

    void error(const std::string& msg);
    void fatalerror(const std::string& msg);

    NodeState getState(){ return _nodeState;};
    bool isInRunning(){return _nodeState==RUNNING;}
    bool isInStopped(){return _nodeState==STOPPED;}
    bool isInInit(){return _nodeState==PREINIT;}
    bool isInError(){return _nodeState==INERROR;}
    bool isInFatalError(){return _nodeState==INFATALERROR;}

  private:


    /**
     * @brief private default constructor.
     * This is declared as private to enforce deriving classes to call an available public constructor and enforce that the ROS services are properly initialised.
     */
    CarosNodeServiceInterface();

    /**
     * @brief internal GripperServiceInterface function to properly advertise the services using ROS and setup the callbacks.
     * This should not be called directly.
     */
    void initCarosNode();

    /* - these functions should be grouped together in the documentation t shortly describe that these are converting from ROS types to e.g. RobWork types according to what the interface expects - */
    bool stopHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    bool startHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    bool configureHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    bool cleanupHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool recoverHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    void changeState(NodeState state);
  private:
    rw::common::Ptr<ros::NodeHandle> _nodeHnd;


    ros::Publisher _nodeStatePublisher;

    ros::ServiceServer _srvStop;
    ros::ServiceServer _srvStart;
    ros::ServiceServer _srvConfigure;
    ros::ServiceServer _srvCleanup;
    ros::ServiceServer _srvRecover;

    NodeState _nodeState, _previousState;
    std::string _carosNodeName, _errorMsg;

  };
} // namespace
#endif
