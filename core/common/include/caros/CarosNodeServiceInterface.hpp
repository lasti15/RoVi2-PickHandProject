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
         * @param[in] loopRateFrequency Optional parameter that specifies the frequency [Hz] of this ROS node - see setLoopRateFrequency() for more information.
         */
        CarosNodeServiceInterface(const ros::NodeHandle& nodehandle, const double loopRateFrequency = 30);

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

        virtual void initLoopHook() = 0;
        virtual void stoppedLoopHook() = 0;
        virtual void runLoopHook() = 0;
        virtual void errorLoopHook() = 0;
        virtual void fatalErrorLoopHook() = 0;

        void error(const std::string& msg, const int64_t errorCode = CAROS_NODE_NO_ERROR_CODE_SUPPLIED);
        void fatalError(const std::string& msg, const int64_t errorCode = CAROS_NODE_NO_ERROR_CODE_SUPPLIED);

        NodeState getState() { return _nodeState; };
        bool isInRunning() { return _nodeState == RUNNING; }
        bool isInStopped() { return _nodeState == STOPPED; }
        bool isInInit() { return _nodeState == PREINIT; }
        bool isInError() { return _nodeState == INERROR; }
        bool isInFatalError() { return _nodeState == INFATALERROR; }

        /**
         * @brief Change the frequency of this ROS node.
         *
         * @param[in] loopRate The new frequency [Hz].
         *
         * Change how often this node is supposed to execute ROS service callbacks and publish CAROS node messages.
         * A very small value or a negative value will (according to the current roscpp implementation) cause this ROS node to process the service callbacks and publishers as fast as possible.
         */
        void setLoopRateFrequency(const double frequency);

        /**
         * @brief Get the frequency of this ROS node.
         *
         * @returns The frequency [Hz].
         */
        double getLoopRateFrequency() { return _loopRateFrequency; }

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

        /* TODO:
         * Should the _loopRateFrequency be settable through the CarosNodeServiceInterface that is exposed as ROS services?
         */
        double _loopRateFrequency;
        ros::Rate _loopRate;

        std::string _errorMsg;
        /* Using int64_t because it's highly related with the type specified in the caros_common::CarosNodeState message */
        int64_t _errorCode;

    };
} // namespace
#endif
