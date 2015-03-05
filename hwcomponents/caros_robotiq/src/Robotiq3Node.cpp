#include <caros/Robotiq3Node.hpp>

#include <rw/rw.hpp>

USE_ROBWORK_NAMESPACE

using namespace robwork;


Robotiq3Node::Robotiq3Node(const ros::NodeHandle& nodehandle):
            caros::CarosNodeServiceInterface(nodehandle),
            caros::GripperServiceInterface(nodehandle),
            _lastQ(4,0,0,0,0),
            _robotiq3(NULL),
            _nodeHandle(nodehandle)

{
    /* Currently nothing specific should happen */
}

Robotiq3Node::~Robotiq3Node() {
    if (!cleanupGripperService()) {
        ROS_ERROR_STREAM("cleanupGripperService() failed.");
    }

    if (_robotiq3 != NULL) {
        if (_robotiq3->isConnected()) {
            ROS_DEBUG_STREAM("Still connected to the Robotiq device - going to stop the device and disconnect.");
            _robotiq3->disconnect();
        }
        _robotiq3 = NULL;
    } else {
        ROS_DEBUG_STREAM("There was no Robotiq device to destroy before deallocating/destroying the Robotci3Node object.");
    }
}


bool Robotiq3Node::activateHook(){
    if (!configureRobotiqDevice()) {
        return false;
    }

    if (!connectToRobotiqDevice()) {
        return false;
    }

    return true;
}

bool Robotiq3Node::recoverHook() {
    /* TODO: */

    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " has not been implemented yet!");
    ROS_BREAK();

    return false;
}

void Robotiq3Node::runLoopHook() {
    try {
        if (_robotiq3 == 0) {
            CAROS_FATALERROR("The Robotiq device is not configured", ROBOTIQ3NODE_INTERNAL_ERROR);
            return;
        }

        if (! _robotiq3->isConnected()) {
            CAROS_ERROR("There is no established connection to the Robotiq device.", ROBOTIQ3NODE_ROBOTIQ_DEVICE_NO_CONNECTION);
            return;
        }

        /************************************************************************
         * Get the time since last time this function was run.
        ************************************************************************/
        ros::Time now = ros::Time::now();
        ros::Duration diff = now-_lastLoopTime;

        /************************************************************************
         * Get current gripper state and split values
         ************************************************************************/
        _robotiq3->getAllStatusCMD();
        Q q = _robotiq3->getQ();
        Q dqcalc = (q-_lastQ)/diff.toSec();
        ROS_DEBUG_STREAM_NAMED("velocity", "Calculated velocity: " << dqcalc);
        Q force = _robotiq3->getQCurrent();
        bool ismov = _robotiq3->isGripperMoving();
        bool isblock = _robotiq3->isGripperBlocked();
        bool isstop = !_robotiq3->isGripperMoving() && !_robotiq3->isGripperBlocked();
        /* FIXME: hardcoded isEstop value */
        bool isEstop = false;
        publishState(q,dqcalc,force,ismov,isblock,isstop,isEstop);

        _lastQ = q;
        _lastLoopTime = now;

    } catch(const rw::common::Exception& exp) {
        CAROS_ERROR(exp.what(), ROBOTIQ3NODE_INTERNAL_ERROR);
        return;
    }
}

void Robotiq3Node::errorLoopHook(){
    /* Stop the Robotiq's current action(s) */
    if (_robotiq3 == 0) {
        ROS_DEBUG_STREAM("The Robotiq device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
    } else {
        _robotiq3->stopCmd();
        _robotiq3->disconnect();
    }
}

void Robotiq3Node::fatalErrorLoopHook(){
    /* Stop the Robotiq's current action(s) */
    if (_robotiq3 == 0) {
        ROS_DEBUG_STREAM("The Robotiq device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
    } else {
        _robotiq3->stopCmd();
        _robotiq3->disconnect();
    }
}


bool Robotiq3Node::configureRobotiqDevice() {
    if (_robotiq3 != 0) {
        /* Could also just silently return true or false and ignore the error that configure is being invoked twice... */
        CAROS_FATALERROR("The Robotiq device is already active - trying to configure an already configured Robotiq3 node is a bug!", ROBOTIQ3NODE_ROBOTIQ_DEVICE_ALREADY_ACTIVE);
        return false;
    }

    /* Fetch parameters (if any) or use the defaults */
    _nodeHandle.param("ip", _ip, std::string("192.168.100.21") );
    _nodeHandle.param("port", _port, 502);

    // TODO: Verify that the chosen parameters are valid?

    _robotiq3 = ownedPtr( new rwhw::Robotiq3() );

    if (!configureGripperService()) {
        CAROS_FATALERROR("The CAROS GripperService could not be configured correctly.", ROBOTIQ3NODE_CAROS_GRIPPER_SERVICE_CONFIGURE_FAIL);
        return false;
    }

    // TODO Implement the limit functions for Robotiq
//    /* Outputting information on supported value ranges */
//    std::pair<rw::math::Q, rw::math::Q> positionLimits = _robotiq->getPosLimits();
//    rw::math::Q velocityLimits = _robotiq->getVelLimits();
//    /* There's also getAccLimits() */
//    rw::math::Q currentLimits = _robotiq->getCurrentLimits();
//
//    ROS_ERROR_STREAM_COND(positionLimits.first.size() != positionLimits.second.size(), "The sizes of the Q's in the position limit pair are not equal; first contains " << positionLimits.first.size() << " and second contains " << positionLimits.second.size() << " elements.");
//
//    ROS_INFO_STREAM("Lower position limits: " << positionLimits.first);
//    ROS_INFO_STREAM("Upper position limits: " << positionLimits.second);
//    ROS_INFO_STREAM("Velocity limits: " << velocityLimits);
//    ROS_INFO_STREAM("Current limits: " << currentLimits);

    /* TODO: Debug information on what was configured accordingly to the parameter server? */
    return true;
}


bool Robotiq3Node::connectToRobotiqDevice() {
    if (_robotiq3 == 0) {
        CAROS_FATALERROR("The Robotiq device is not configured", ROBOTIQ3NODE_INTERNAL_ERROR);
        return false;
    }

    if (_robotiq3->isConnected()) {
        ROS_ERROR_STREAM("'" << __PRETTY_FUNCTION__ << "' invoked even though a connection to the Robotiq device has already been established - this is a bug!");
        return false;
    }

    /* Connect according to configured parameters */
    if (!_robotiq3->connect(_ip, _port)) {
        CAROS_FATALERROR("The robotiq hand was not able to connect to" << _ip << " : " << _port, CONNECTION_ERROR);
        return false;
    }

    /* Verify that the connection to the Robotiq device has been established - this eliminates the need for verifying that the _robotiq->connect() function calls actually succeed */
    if (! _robotiq3->isConnected()) {
        /* Something went wrong when connecting */
        CAROS_FATALERROR("Failed to properly connect to the Robotiq device.", ROBOTIQ3NODE_ROBOTIQ_DEVICE_CONNECT_FAILED);
        return false;
    }

    return true;
}

/************************************************************************
 * GripperServiceInterface
 ************************************************************************/
/* Note:
 * The checks isInRunning(), (_robotiq == 0) and (! _robotiq->isConnected()) are not placed in one common function, because the CAROS_ERROR and CAROS_FATALERROR macros are using source code lines to sort of pinpoint the "faulty" function.
 * When a more appropriate method is found that can reduce this code duplication, then it should be implemented! (A preprocessor code generating macro is not exactly a nice and easily maintainable solution)
 */
bool Robotiq3Node::moveQ(const rw::math::Q& q){
    if (!isInRunning()) {
        ROS_WARN_STREAM("Not in running state!");
        return false;
    }

    if (_robotiq3 == 0) {
        CAROS_FATALERROR("The Robotiq device is not configured.", ROBOTIQ3NODE_NO_ROBOTIQ_DEVICE);
        return false;
    }

    if (! _robotiq3->isConnected()) {
        CAROS_ERROR("There is no established connection to the Robotiq device.", ROBOTIQ3NODE_ROBOTIQ_DEVICE_NO_CONNECTION);
        return false;
    }

    try {
        _robotiq3->moveCmd(q);
        _lastCmd = MOVE;
    } catch (const rw::common::Exception& exp) {
        CAROS_ERROR(exp.what(), ROBOTIQ3NODE_INTERNAL_ERROR);
        return false;
    }
    return true;
}

bool Robotiq3Node::gripQ(const rw::math::Q& q){
    if (!isInRunning()) {
        ROS_WARN_STREAM("Not in running state!");
        return false;
    }

    if (_robotiq3 == 0) {
        CAROS_FATALERROR("The Robotiq device is not configured.", ROBOTIQ3NODE_NO_ROBOTIQ_DEVICE);
        return false;
    }

    if (! _robotiq3->isConnected()) {
        CAROS_ERROR("There is no established connection to the Robotiq device.", ROBOTIQ3NODE_ROBOTIQ_DEVICE_NO_CONNECTION);
        return false;
    }

    try {
        _robotiq3->moveCmd(q);
        _lastCmd = GRIP;
    } catch (const rw::common::Exception& exp) {
        CAROS_ERROR(exp.what(), ROBOTIQ3NODE_INTERNAL_ERROR);
        return false;
    }
    return true;
}

bool Robotiq3Node::setForceQ(const rw::math::Q& q){
    if (!isInRunning()) {
        ROS_WARN_STREAM("Not in running state!");
        return false;
    }

    if (_robotiq3 == 0) {
        CAROS_FATALERROR("The Robotiq device is not configured.", ROBOTIQ3NODE_NO_ROBOTIQ_DEVICE);
        return false;
    }

    if (! _robotiq3->isConnected()) {
        CAROS_ERROR("There is no established connection to the Robotiq device.", ROBOTIQ3NODE_ROBOTIQ_DEVICE_NO_CONNECTION);
        return false;
    }

    try {
        _robotiq3->setTargetQForce(q);
    } catch (const rw::common::Exception& exp) {
        CAROS_ERROR(exp.what(), ROBOTIQ3NODE_INTERNAL_ERROR);
        return false;
    }
    return true;
}

bool Robotiq3Node::setVelocityQ(const rw::math::Q& q){
    if (!isInRunning()) {
        ROS_WARN_STREAM("Not in running state!");
        return false;
    }

    if (_robotiq3 == 0) {
        CAROS_FATALERROR("The Robotiq device is not configured.", ROBOTIQ3NODE_NO_ROBOTIQ_DEVICE);
        return false;
    }

    if (! _robotiq3->isConnected()) {
        CAROS_ERROR("There is no established connection to the Robotiq device.", ROBOTIQ3NODE_ROBOTIQ_DEVICE_NO_CONNECTION);
        return false;
    }

    try {
        _robotiq3->setTargetQVel(q);
    } catch (const rw::common::Exception& exp) {
        CAROS_ERROR(exp.what(), ROBOTIQ3NODE_INTERNAL_ERROR);
        return false;
    }

    return true;
}

bool Robotiq3Node::stopMovement() {
    if (!isInRunning()) {
        ROS_WARN_STREAM("Not in running state!");
        return false;
    }

    if (_robotiq3 == 0) {
        CAROS_FATALERROR("The Robotiq device is not configured.", ROBOTIQ3NODE_NO_ROBOTIQ_DEVICE);
        return false;
    }

    if (! _robotiq3->isConnected()) {
        CAROS_ERROR("There is no established connection to the Robotiq device.", ROBOTIQ3NODE_ROBOTIQ_DEVICE_NO_CONNECTION);
        return false;
    }

    try {
        _robotiq3->stopCmd();
    } catch (const rw::common::Exception& exp) {
        CAROS_ERROR(exp.what(), ROBOTIQ3NODE_INTERNAL_ERROR);
        return false;
    }

    return true;
}

