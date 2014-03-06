#include <caros/CarosNodeServiceInterface.hpp>

#include <caros_common/CarosNodeState.h>

using namespace caros;

namespace {
    static std::string CarosStateString[] = {"PREINIT","STOPPED","RUNNING","INERROR","INFATALERROR"};
}

CarosNodeServiceInterface::CarosNodeServiceInterface(rw::common::Ptr<ros::NodeHandle> nodehandle):
    _nodeHnd( rw::common::ownedPtr(new ros::NodeHandle(nodehandle->getNamespace() + "/carosnode" ) ) ),
    _nodeState(PREINIT),
    _carosNodeName( nodehandle->getNamespace() + "/carosnode" )
{
    initCarosNode();
}

CarosNodeServiceInterface::CarosNodeServiceInterface(const std::string& service_name):
        _nodeHnd( rw::common::ownedPtr(new ros::NodeHandle(service_name+"/carosnode") ) ),
        _nodeState(PREINIT),
        _carosNodeName(service_name+"/carosnode")
{
    initCarosNode();
}

CarosNodeServiceInterface::~CarosNodeServiceInterface(){

}



void CarosNodeServiceInterface::start(){

    caros_common::CarosNodeState state;
    ros::Rate _loopRate(30);
    while (ros::ok()) {
        // publish state with optional error msg. low rate
        state.state = CarosStateString[_nodeState];
        state.inError = _nodeState==INERROR || _nodeState==INFATALERROR;
        if(state.inError)
            state.errorMsg = _errorMsg;
        state.changedEvent = false;
        _nodeStatePublisher.publish( state );

        if(_nodeState==RUNNING){
            runloopHook();
        } else if(_nodeState==STOPPED){
            // publish state
            stoppedLoopHook();
        } else if(_nodeState==INERROR){
            // publish error
            errorLoopHook();
        } else if(_nodeState==INFATALERROR){
            // publish error
            fatalerrorLoopHook();
        } else if(_nodeState==PREINIT){
            // initloop
            initLoopHook();
        }
        ros::spinOnce();
        _loopRate.sleep();
    }
}

bool CarosNodeServiceInterface::startNode(){
    // can only be called in stopped
    if(_nodeState != STOPPED && _nodeState != PREINIT){
        error("Start can only be called from STOPPED state. Currently the node is in " + CarosStateString[_nodeState]);
        return false;
    }
    if(_nodeState==PREINIT)
        configureNode();
    try{
        startHook();
        changeState(RUNNING);
    } catch(const std::exception& e){
        error( e.what() );
        return false;
    } catch(...){
        fatalerror("Unknown exception thrown in node...");
        return false;
    }
    return true;
}

bool CarosNodeServiceInterface::stopNode(){
    // can only be called in running
    if(_nodeState != RUNNING){
        error("Stop can only be called from RUNNING state. Currently the node is in " + CarosStateString[_nodeState]);
        return false;
    }
    try{
        stopHook();
        changeState(STOPPED);
    } catch(const std::exception& e){
        error(e.what());
        return false;
    } catch(...){
        fatalerror("Unknown exception thrown in node...");
        return false;
    }
    return true;
}


bool CarosNodeServiceInterface::recoverNode(){
    // can only be called in running
    if(_nodeState != INERROR){
        error("Recover can only be called from ERROR state. Currently the node is in " + CarosStateString[_nodeState]);
        return false;
    }
    try{
        recoverHook();
        changeState(_previousState);
    } catch(const std::exception& e){
        error(e.what());
        return false;
    } catch(...){
        fatalerror("Unknown exception thrown in node...");
        return false;
    }
    return true;
}

void CarosNodeServiceInterface::error(const std::string& msg){
    ROS_ERROR_STREAM("CarosNodeError: " << msg);
    // publish error
    _errorMsg = msg;
    // handle error switch state
    changeState(INERROR);
}

void CarosNodeServiceInterface::fatalerror(const std::string& msg){
    ROS_ERROR_STREAM("CarosNodeFatalError: " << msg);
    // publish error
    _errorMsg = msg;
    // handle error switch state
    changeState(INFATALERROR);
}

void CarosNodeServiceInterface::changeState(NodeState newstate){
    if(newstate!=_nodeState)
        _previousState = _nodeState;
    _nodeState = newstate;

    caros_common::CarosNodeState state;
    state.state = CarosStateString[_nodeState];
    state.inError = _nodeState==INERROR || _nodeState==INFATALERROR;
    if(state.inError)
        state.errorMsg = _errorMsg;
    state.changedEvent = true;

    _nodeStatePublisher.publish( state );
}

bool CarosNodeServiceInterface::configureNode()
{
    if (_nodeState != PREINIT) {
        error("Configure can only be called from INIT state. Currently the node is in " + CarosStateString[_nodeState]);
        return false;
    }
    try {
        configureHook();
        changeState(STOPPED);
    } catch (const std::exception& e) {
        error(e.what());
        return false;
    } catch (...) {
        fatalerror("Unknown exception thrown in node...");
        return false;
    }
    return true;
}

bool CarosNodeServiceInterface::cleanupNode(){
    if(_nodeState==RUNNING)
        stopNode();

    try{
        cleanupHook();
        changeState(PREINIT);
    } catch(const std::exception& e){
        error(e.what());
        return false;
    } catch(...){
        fatalerror("Unknown exception thrown in node...");
        return false;
    }
    return true;
}

void CarosNodeServiceInterface::initCarosNode(){
    _nodeStatePublisher = _nodeHnd->advertise<caros_common::CarosNodeState>("CarosNodeState", 1);

    _srvStop  = _nodeHnd->advertiseService("stop", &CarosNodeServiceInterface::stopHandle, this);
    _srvStart = _nodeHnd->advertiseService("start", &CarosNodeServiceInterface::startHandle, this);
    _srvConfigure = _nodeHnd->advertiseService("configure", &CarosNodeServiceInterface::configureHandle, this);
    _srvCleanup = _nodeHnd->advertiseService("cleanup", &CarosNodeServiceInterface::cleanupHandle, this);
    _srvRecover = _nodeHnd->advertiseService("recover", &CarosNodeServiceInterface::recoverHandle, this);
}

/* - these functions should be grouped together in the documentation t shortly describe that these are converting from ROS types to e.g. RobWork types according to what the interface expects - */
bool CarosNodeServiceInterface::stopHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
    return stopNode();
}

bool CarosNodeServiceInterface::startHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
    return startNode();
}

bool CarosNodeServiceInterface::configureHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    return configureNode();
}

bool CarosNodeServiceInterface::cleanupHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
    return cleanupNode();
}

bool CarosNodeServiceInterface::recoverHandle(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
    return recoverNode();
}
