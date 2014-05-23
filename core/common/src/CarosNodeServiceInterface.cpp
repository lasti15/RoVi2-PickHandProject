#include <caros/CarosNodeServiceInterface.hpp>
#include <caros_common/CarosNodeState.h>

#include <ros/ros.h>

#include <string>

/************************************************************************
 * TODO:
 * - Should the publish rate on the NodeState be lowered to maybe 1 or
 *   2 Hz when in an error state?
 * - Make it possible to configure the _loopRate through the parameter
 *   server.
 ************************************************************************/

namespace caros {

    namespace {
        static std::string CarosStateString[] = {"PREINIT","STOPPED","RUNNING","INERROR","INFATALERROR"};
    }

    CarosNodeServiceInterface::CarosNodeServiceInterface(const ros::NodeHandle& nodehandle):
        _nodeHandle(nodehandle, CAROS_NODE_SERVICE_INTERFACE_SUB_NAMESPACE),
        _nodeState(PREINIT)
    {
        if (initCarosNode()) {
            /* TODO:
             * FIXME:
             *   How to react on an unsuccessful initialisation? - zombie object or throw an exception
             */
        }
    }

    CarosNodeServiceInterface::~CarosNodeServiceInterface() {
        /* Nothing to destroy */
    }

    void CarosNodeServiceInterface::start() {
        caros_common::CarosNodeState state;
        ros::Rate _loopRate(30);
        while (ros::ok()) {
            ros::spinOnce();

            if (_nodeState == RUNNING) {
                runLoopHook();
            } else if (_nodeState == STOPPED) {
                stoppedLoopHook();
            } else if (_nodeState == PREINIT) {
                initLoopHook();
            }

            /* Process errors if any occured */
            if (_nodeState == INERROR) {
                errorLoopHook();
            }
            /* Also process fatal error state if an error should become a fatal error */
            if (_nodeState == INFATALERROR) {
                fatalErrorLoopHook();
            }

            publishNodeState();

            /* Sleep to run approximately at the specified Hz */
            _loopRate.sleep();
        }
    }

    /* TODO: These methods can just be converted to void, as the return value will always be true (except for cleanupNode()) */
    bool CarosNodeServiceInterface::configureNode() {
        if (_nodeState != PREINIT) {
            ROS_ERROR_STREAM("Configure can only be called from " << CarosStateString[PREINIT] << " state. The node was in " << CarosStateString[_nodeState] << ".");
            return false;
        }

        if (configureHook()) {
            changeState(STOPPED);
        }

        return true;
    }

    bool CarosNodeServiceInterface::cleanupNode() {
        if (_nodeState == PREINIT) {
            ROS_WARN_STREAM("It does not make much sense to invoke cleanupNode() when in the " << CarosStateString[_nodeState] << " state!");
            return false;
        }

        if (_nodeState == RUNNING) {
            stopNode();
        }

        if (cleanupHook()) {
            changeState(PREINIT);
        }

        return true;
    }

    bool CarosNodeServiceInterface::startNode() {
        // can only be called in stopped
        if (_nodeState != STOPPED) {
            ROS_ERROR_STREAM("Start can only be called when in " << CarosStateString[STOPPED] << " state. The node was in " << CarosStateString[_nodeState] << ".");
            return false;
        }

        if (startHook()) {
            changeState(RUNNING);
        }

        return true;
    }

    bool CarosNodeServiceInterface::stopNode() {
        /* TODO: Should this be callable when in the error state? */
        if (_nodeState != RUNNING) {
            ROS_ERROR_STREAM("Stop can only be called from " << CarosStateString[RUNNING] << " state. The node was in " << CarosStateString[_nodeState] << ".");
            return false;
        }

        if (stopHook()) {
            changeState(STOPPED);
        }

        return true;
    }

    bool CarosNodeServiceInterface::recoverNode() {
        // can only be called when in error state
        if(_nodeState != INERROR) {
            ROS_ERROR_STREAM("Recover can only be called from " << CarosStateString[INERROR] << " state. The node was in " << CarosStateString[_nodeState] << ".");
            return false;
        }
        
        if (recoverHook()) {
            changeState(_previousState);
        }

        return true;
    }

    void CarosNodeServiceInterface::error(const std::string& msg) {
        ROS_ERROR_STREAM("CarosNodeError: " << msg);
        /* keep a copy of the error message so it can be published */
        _errorMsg = msg;
        _errorCode = CAROS_NODE_NO_ERROR_CODE_SUPPLIED;
        changeState(INERROR);
    }

    void CarosNodeServiceInterface::fatalError(const std::string& msg) {
        ROS_ERROR_STREAM("CarosNodeFatalError: " << msg);
        /* keep a copy of the (fatal) error message so it can be published */
        _errorMsg = msg;
        _errorCode = CAROS_NODE_NO_ERROR_CODE_SUPPLIED;
        changeState(INFATALERROR);
    }

    void CarosNodeServiceInterface::error(const std::string& msg, const int64_t errorCode) {
        ROS_ERROR_STREAM("CarosNodeError: " << msg << "; error code: " << errorCode);
        /* keep a copy of the error message so it can be published */
        _errorMsg = msg;
        _errorCode = errorCode;
        changeState(INERROR);
    }

    void CarosNodeServiceInterface::fatalError(const std::string& msg, const int64_t errorCode) {
        ROS_ERROR_STREAM("CarosNodeFatalError: " << msg << "; error code: " << errorCode);
        /* keep a copy of the (fatal) error message so it can be published */
        _errorMsg = msg;
        _errorCode = errorCode;
        changeState(INFATALERROR);
    }

    void CarosNodeServiceInterface::changeState(const NodeState newstate) {
        if (newstate != _nodeState) {
            _previousState = _nodeState;
        }
        _nodeState = newstate;
        publishNodeState(true);
    }

    bool CarosNodeServiceInterface::initCarosNode(){
        if (_nodeStatePublisher || _srvStop || _srvStart || _srvConfigure || _srvCleanup || _srvRecover) {
            ROS_WARN_STREAM("Reinitialising one or more CarosNodeServiceInterface services or publishers. If this is not fully intended then this should be considered a bug!");
        }

        _nodeStatePublisher = _nodeHandle.advertise<caros_common::CarosNodeState>("CarosNodeState", 1);
        ROS_ERROR_STREAM_COND(!_nodeStatePublisher, "The CarosNodeState publisher is empty!");

        _srvStop  = _nodeHandle.advertiseService("stop", &CarosNodeServiceInterface::stopHandle, this);
        ROS_ERROR_STREAM_COND(!_srvStop , "The stop service is empty!");

        _srvStart = _nodeHandle.advertiseService("start", &CarosNodeServiceInterface::startHandle, this);
        ROS_ERROR_STREAM_COND(!_srvStart, "The start service is empty!");

        _srvConfigure = _nodeHandle.advertiseService("configure", &CarosNodeServiceInterface::configureHandle, this);
        ROS_ERROR_STREAM_COND(!_srvConfigure, "The configure service is empty!");

        _srvCleanup = _nodeHandle.advertiseService("cleanup", &CarosNodeServiceInterface::cleanupHandle, this);
        ROS_ERROR_STREAM_COND(!_srvCleanup, "The cleanup service is empty!");

        _srvRecover = _nodeHandle.advertiseService("recover", &CarosNodeServiceInterface::recoverHandle, this);
        ROS_ERROR_STREAM_COND(!_srvRecover, "The recover service is empty!");

        if (_nodeStatePublisher && _srvStop && _srvStart && _srvConfigure && _srvCleanup && _srvRecover) {
            /* Everything seems to be properly initialised */
        } else {
            ROS_FATAL_STREAM("One or more of the ROS publishers or services could not be properly initialised.");
            return false;
        }

        return true;
    }

/* - these functions should be grouped together in the documentation to shortly describe that these are converting from ROS types to e.g. RobWork types according to what the interface expects - */
    bool CarosNodeServiceInterface::stopHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
        return stopNode();
    }

    bool CarosNodeServiceInterface::startHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
        return startNode();
    }

    bool CarosNodeServiceInterface::configureHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
        return configureNode();
    }

    bool CarosNodeServiceInterface::cleanupHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
        return cleanupNode();
    }

    bool CarosNodeServiceInterface::recoverHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
        return recoverNode();
    }

    void CarosNodeServiceInterface::publishNodeState(const bool stateChanged) {
        caros_common::CarosNodeState state;
        state.state = CarosStateString[_nodeState];
        state.inError = _nodeState == INERROR || _nodeState == INFATALERROR;
        if (state.inError) {
            state.errorMsg = _errorMsg;
            state.errorCode = _errorCode;
        }
        /* The errorMsg is not being cleared when the state nolonger is in error, this is intended to provide a minor error log / history of errors.
         * TODO: Provide a history/log of the last N errors together with some info such as a timestamp (maybe how long the node was in the error state) and similar.
         */

        state.changedEvent = stateChanged;

        _nodeStatePublisher.publish( state );
    }
} // namespace caros
