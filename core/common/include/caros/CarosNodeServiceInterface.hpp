#ifndef CAROS_CAROSNODESERVICEINTERFACE_HPP_
#define CAROS_CAROSNODESERVICEINTERFACE_HPP_

#include <rw/math/Q.hpp>
#include <rw/common/Ptr.hpp>

#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include <string>

#define CAROS_NODE_SERVICE_INTERFACE_SUB_NAMESPACE "caros_node"

/************************************************************************
 * TODO:
 * - Add apidoc documentation
 ************************************************************************/

//enum CAROS_NODE_ERRORCODE { CAROS_NODE_NO_ERROR_CODE_SUPPLIED = 0 }

/**
 * @brief Emit an caros node error. It will also be emitted to ROS_ERROR
 *
 * \b ostreamExpression is an expression that is fed to an output stream. Example:
\code
CAROS_ERROR("The value of x is " << x << ". x should be less than zero.", 2);
\endcode
 *
 */
#define CAROS_ERROR(ostreamExpression, errorCode) do { \
    std::stringstream RW__stream;                                           \
    RW__stream << ostreamExpression;                                        \
    ROS_ERROR_STREAM("CarosNodeError: " << RW__stream.str() << "; error code: " << errorCode); \
    error(RW__stream.str(), errorCode );                                     \
    } while (0)

/**
 * @brief Emit an caros node fatal error. It will also be emitted to ROS_ERROR
 *
 * \b ostreamExpression is an expression that is fed to an output stream. Example:
\code
CAROS_ERROR("The value of x is " << x << ". x should be less than zero.", 2);
\endcode
 *
 */
#define CAROS_FATALERROR(ostreamExpression, errorCode) do { \
    std::stringstream RW__stream;                                           \
    RW__stream << ostreamExpression;                                        \
    ROS_ERROR_STREAM("CarosNodeError: " << RW__stream.str() << "; error code: " << errorCode); \
    fatalerror(RW__stream.str(), errorCode );                                     \
    } while (0)


namespace caros {
    enum CAROS_NODE_ERRORCODE { CAROS_NODE_NO_ERROR_CODE_SUPPLIED = 0 };

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
        CarosNodeServiceInterface(const ros::NodeHandle& nodehandle);

        /**
         * @brief virtual destructor
         */
        virtual ~CarosNodeServiceInterface();

        void start();

        bool configureNode();
        /* TODO: Mention/document that the cleanupNode() will also invoke stopNode if the current state is RUNNING */
        bool cleanupNode();
        bool startNode();
        bool stopNode();
        bool recoverNode();

        std::string getErrorMsg();

        // possible node states
        typedef enum {PREINIT,STOPPED,RUNNING,INERROR,INFATALERROR} NodeState;

    protected:

        virtual bool configureHook() = 0;
        virtual bool cleanupHook() = 0;
        virtual bool startHook() = 0;
        virtual bool stopHook() = 0;
        virtual bool recoverHook() = 0;

        virtual void runLoopHook() = 0;

        virtual void initLoopHook(){};
        virtual void stoppedLoopHook(){};
        virtual void errorLoopHook(){};
        virtual void fatalErrorLoopHook(){};

        void error(const std::string& msg, const int64_t errorCode = CAROS_NODE_NO_ERROR_CODE_SUPPLIED);
        void fatalError(const std::string& msg, const int64_t errorCode = CAROS_NODE_NO_ERROR_CODE_SUPPLIED);

        NodeState getState() { return _nodeState; };
        bool isInRunning() { return _nodeState == RUNNING; }
        bool isInStopped() { return _nodeState == STOPPED; }
        bool isInInit() { return _nodeState == PREINIT; }
        bool isInError() { return _nodeState == INERROR; }
        bool isInFatalError() { return _nodeState == INFATALERROR; }

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
        bool initCarosNode();

        /* - These functions should be grouped together in the documentation to shortly describe that these are converting from ROS types to e.g. RobWork types according to what the interface expects - */
        bool stopHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        bool startHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        bool configureHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        bool cleanupHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
        bool recoverHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        void changeState(NodeState state);

        void publishNodeState(const bool stateChanged = false);

    private:
        ros::NodeHandle _nodeHandle;

        ros::Publisher _nodeStatePublisher;

        ros::ServiceServer _srvStop;
        ros::ServiceServer _srvStart;
        ros::ServiceServer _srvConfigure;
        ros::ServiceServer _srvCleanup;
        ros::ServiceServer _srvRecover;

        NodeState _nodeState, _previousState;
        std::string _errorMsg;
        /* Using int64_t because it's highly related with the type specified in the caros_common::CarosNodeState message */
        int64_t _errorCode;

    };
} // namespace
#endif
