#include <caros/UniversalRobots.hpp>

#include <caros/common.h>

#include <rw/invkin/IKMetaSolver.hpp>
#include <rw/math/MetricFactory.hpp>

#include <ros/assert.h>

#include <string>

using namespace caros;

#define URRT_PORT 30003
#define UR_PORT 30001
#define WRENCHTOPIC_QUEUE_SIZE 5
#define WRENCH_DATA_QUEUE_MAX_ALLOWED_NUMBER_OF_ELEMENTS 3
#define URRTDATA_QACTUAL_SIZE 6

UniversalRobots::UniversalRobots(const ros::NodeHandle& nodehandle, rw::models::WorkCell::Ptr workcell):
    CarosNodeServiceInterface(nodehandle),
    SerialDeviceServiceInterface(nodehandle),
    URServiceInterface(nodehandle),
    _nodehandle(nodehandle),
    _workcell(workcell),
    _device(NULL),
    _ftFrame(NULL),
    _useFTCollisionDetection(false)
{
/* Currently nothing specific should happen */
}

UniversalRobots::~UniversalRobots() {
    /* Nothing special to destroy */
}

bool UniversalRobots::activateHook() {
    /* TODO:
     * - Maybe split all this configuration/activation logic up into a configure and connect step, like what has been done for the SDH node.
     * ^- Should the fetching of the parameters be moved into its own function, instead of having this big activateHook() function?
     */
    /************************************************************************
     * Parameters
     ************************************************************************/
    std::string deviceName;
    if (! _nodehandle.getParam("deviceName", deviceName)) {
        CAROS_FATALERROR("The parameter '" << _nodehandle.getNamespace() << "/deviceName' was not present on the parameter server! This parameter has to be specified for this node to work properly.", URNODE_MISSING_PARAMETER);
        return false;
    }
    /* TODO:
     * FIXME:
     * Should ftFrameName default to WORLD or should the ROS parameter be explicitly set if _ftFrame is to be used (i.e. the moveLinFC function controls whether _ftFrame is NULL or not before doing anything...)?
     */
    std::string ftFrameName;
    _nodehandle.param("FTFrame", ftFrameName, std::string("WORLD"));
    std::string ip;
    if (! _nodehandle.getParam("IP", ip)) {
        CAROS_FATALERROR("The parameter '" << _nodehandle.getNamespace() << "/IP' was not present on the parameter server! This parameter has to be specified for this node to work properly.", URNODE_MISSING_PARAMETER);
        return false;
    }

    std::string callbackIP;
    if (! _nodehandle.getParam("callbackIP", callbackIP)) {
        CAROS_FATALERROR("The parameter '" << _nodehandle.getNamespace() << "/callbackIP' was not present on the parameter server! This parameter has to be specified for this node to work properly.", URNODE_MISSING_PARAMETER);
        return false;
    }

    std::string callbackPort;
    if (! _nodehandle.getParam("callbackPort", callbackPort)) {
        CAROS_FATALERROR("The parameter '" << _nodehandle.getNamespace() << "/callbackPORT' was not present on the parameter server! This parameter has to be specified for this node to work properly.", URNODE_MISSING_PARAMETER);
        return false;
    }

    std::string wrenchTopic;
    _nodehandle.param("WrenchTopic", wrenchTopic, std::string());    
    if (!wrenchTopic.empty()) {
        _subFTData = _nodehandle.subscribe(wrenchTopic, WRENCHTOPIC_QUEUE_SIZE, &UniversalRobots::addFTData, this);
        if (! _subFTData) {
            CAROS_FATALERROR("The subscriber for the topic '" << wrenchTopic << "' could not be properly created.", URNODE_FAULTY_SUBSCRIBER);
            return false;
        }
    }

    /* TODO:
     * [ as of this writing ] There is no way to get a working workcell without restarting this URNode, as the functionality to obtain a workcell from the ROS parameter server is placed in ur_main...
     * ^- Consider if this should cause this node to just go into a FATAL_ERROR state instead and make it possible to obtain the workcell from the parameter server through the recover functionality or simply by restarting the node - instead of having the node quit/restart without ending up in the CAROS FATAL ERROR?
     */
    if (_workcell == NULL) {
        CAROS_FATALERROR("No workcell was provided!", URNODE_MISSING_WORKCELL);
        return false;
    }
    
    ROS_ASSERT(_workcell != NULL);
    ROS_DEBUG_STREAM("Looking for the device '" << deviceName << "' in the workcell.");
    _device = _workcell->findDevice(deviceName);
    if (_device == NULL) {
        CAROS_FATALERROR("Unable to find device " << deviceName << " in the loaded workcell", URNODE_NO_SUCH_DEVICE);
        return false;
    }
    
    ROS_DEBUG_STREAM("Looking for the frame '" << ftFrameName << "' in the workcell.");
    _ftFrame = _workcell->findFrame(ftFrameName);
    if (_ftFrame == NULL) {
        /* TODO:
         * Should this be a fatal error or should it just be a warning and then have the actual functionality supporting this (i.e. moveLinFC) verify that _ftFrame is indeed not NULL?
         */
        CAROS_FATALERROR("Couldn't find the ftFrame '" << ftFrameName << "' in the workcell!", URNODE_NO_SUCH_FRAME);
        return false;
    }
    
    _state = _workcell->getDefaultState();
    _iksolver = rw::common::ownedPtr( new rw::invkin::JacobianIKSolver(_device, _state ) );
    
    _urrt.connect(ip, URRT_PORT);
    _ur.connect(ip, UR_PORT);
    
    unsigned int numericCallbackPort = 0;
    try {
        numericCallbackPort = std::stoul(callbackPort);
    } catch (std::invalid_argument& e) {
        /* The conversion could not be performed */
        /* TODO:
         * Go into some error mode?
         */
    } catch (std::out_of_range& e) {
        /* TODO:
         * Go into some error mode?
         */
    }
    /* TODO:
     * Does the ordering of these two start[Interface](...) functions actually matter?
     */
    _ur.startCommunication(callbackIP, numericCallbackPort);
    _urrt.start();
    
    /* TODO:
     * Where do these weights come from?
     */
    rw::math::Q weights(6);
    weights(0) = 0.85;
    weights(1) = 0.85;
    weights(2) = 0.45;
    weights(3) = 0.30;
    weights(4) = 0.20;
    weights(5) = 0.20;
    _q2cmetric = rw::math::MetricFactory::makeWeightedEuclidean(weights);

    if (!configureURService()) {
        CAROS_FATALERROR("The URService could not be configured correctly.", URNODE_URSERVICE_CONFIGURE_FAIL);
        return false;
    }

    if (!SerialDeviceServiceInterface::configureInterface()) {
        CAROS_FATALERROR("The SerialDeviceService could not be configured correctly.", URNODE_SERIALDEVICESERVICE_CONFIGURE_FAIL);
        return false;
    }

    
    return true;
}

bool UniversalRobots::recoverHook() {
    /* TODO:
     * Consider just redoing the activateHook() if e.g. a parameter was missing.
     * ^- Anything that should be cleaned up before trying to reactivate/initialise this node?
     */
    return false;
}

void UniversalRobots::runLoopHook() {
    bool urrtHasData = _urrt.hasData();
    bool urHasData = _ur.getPrimaryInterface().hasData();

    if (urrtHasData == false || urHasData == false) {
        ROS_DEBUG_STREAM("Missing some data from UR: urrtHasData = '" << urrtHasData << "' and urHasData = '" << urHasData << "'");
        return;
    }

    rwhw::URRTData urrtData = _urrt.getLastData();
    /* TODO:
     * Place the UniversalRobotsData within the rwhw namespace (the file is from RobWorkHardware)
     */
    UniversalRobotsData purData = _ur.getPrimaryInterface().getLastData();

    /* TODO:
     * How much of this information and these messages are to be given as INFO, WARNING or DEBUG?
     */
    ROS_DEBUG_STREAM("E-stop: '" << purData.emergencyStopped << "' Security-stop: '" << purData.securityStopped << "' Running: '" << purData.programRunning << "'");

    auto messages = _ur.getPrimaryInterface().getMessages();
    while (!messages.empty()) {
        ROS_DEBUG_STREAM("UR Message: " << messages.front());
        messages.pop();
    }
    _ur.getPrimaryInterface().clearMessages();


    if (urrtData.qActual.size() == URRTDATA_QACTUAL_SIZE) {
        caros_control_msgs::robot_state robotState;
        _qcurrent = urrtData.qActual;
        robotState.q = caros::toRos(urrtData.qActual);
        robotState.dq = caros::toRos(urrtData.dqActual);
        robotState.header.frame_id = _nodehandle.getNamespace();
        robotState.header.stamp = ros::Time::now();
        robotState.estopped = purData.emergencyStopped;
        if(robotState.estopped){
            /* TODO:
             * Is this data really broken, as stated by a comment in the MARVIN version?
             */
            ROS_DEBUG_STREAM_NAMED("robotState estopped", "Is estopped");
        } else {
            ROS_DEBUG_STREAM_NAMED("robotState estopped", "Is not estopped");
        }

        robotState.isMoving = _ur.isMoving();

        /* TODO:
         * isColliding is hardcoded to be false....
         * ^- Look at the uncommented code in the MARVIN version for hints on how the collision detection was done (if it's desirable to bring back that functionality)
         */
        robotState.isColliding = false;

        /* TODO:
         * Is there a better way to publish robotState - maybe look into the new RWstate that has been added to caros_common.
         */
        SerialDeviceServiceInterface::publish(robotState);
    } else {
        /* TODO:
         * Should this be an error (just ROS error or also a CAROS error - e.g. changing the state of this node to error or fatalerror)?
         */
        ROS_WARN_STREAM("The size of urrtData.qActual is '" << urrtData.qActual.size() << "' but should be '" << URRTDATA_QACTUAL_SIZE << "'!");
    }
}

void UniversalRobots::errorLoopHook() {
    /* TODO:
     * Consider what needs to be done when this node is in error - should any of the _urrt or _ur objects/connections be stopped or just let them continue?
     */
}

void UniversalRobots::fatalErrorLoopHook() {
    /* TODO:
     * Consider what needs to be done when this node is in error - should any of the _urrt or _ur objects/connections be stopped or just let them continue?
     */
}

void UniversalRobots::addFTData(const caros_common_msgs::wrench_data::ConstPtr state) {
    rw::math::Wrench6D<> wrench;
    wrench(0) = state->wrench.force.x;
    wrench(1) = state->wrench.force.y;
    wrench(2) = state->wrench.force.z;
    wrench(3) = state->wrench.torque.x;
    wrench(4) = state->wrench.torque.y;
    wrench(5) = state->wrench.torque.z;

    if (_wrenchDataQueue.size() > WRENCH_DATA_QUEUE_MAX_ALLOWED_NUMBER_OF_ELEMENTS) {
        _wrenchDataQueue.pop();
    }

    _wrenchDataQueue.push(wrench);
}

/************************************************************************
 * URServiceInterface functions
 ************************************************************************/
bool UniversalRobots::servoT(const rw::math::Transform3D<>& target) {
    ROS_DEBUG_STREAM("servoT: " << target);
    _device->setQ(_qcurrent, _state);
    std::vector<rw::math::Q> solutions = _iksolver->solve(target, _state);
    if (solutions.empty())
    {
        ROS_ERROR_STREAM("servoT: Unable to find IK solution for target = '" << target << "' and _qcurrent = '" << _qcurrent << "'");
        return true;
    }

    rw::math::Q closest = solutions.front();

    ROS_DEBUG_STREAM("servoT: Q-configuration being sent to driver: " << closest);
    _ur.servo(closest);
    /* There is no (immediate) feedback from the _ur.servo() function call, so just returning true. */
    return true;
}

bool UniversalRobots::servoQ(const rw::math::Q& q) {
    ROS_DEBUG_STREAM("ServoQ: " << q);

    _ur.servo(q);
    /* There is no (immediate) feedback from the _ur.servo() function call, so just returning true. */
    return true;
}

bool UniversalRobots::forceModeStart(const rw::math::Transform3D<>& refToffset, const rw::math::Q& selection, const rw::math::Wrench6D<>& wrenchTarget, const rw::math::Q& limits)
{
    ROS_DEBUG_STREAM("ForceModeStart arguments begin:");
    ROS_DEBUG_STREAM("refToffset: " << refToffset);
    ROS_DEBUG_STREAM("selection: " << selection);
    ROS_DEBUG_STREAM("wrenchTarget: " << wrenchTarget);
    ROS_DEBUG_STREAM("limits: " << limits);
    ROS_DEBUG_STREAM("ForceModeStart arguments end");

    if (selection.size() != 6) {
        ROS_WARN_STREAM("The number of elements in selection is '" << selection.size() << "' but should be '" << 6 << "'");
        return false;
    }
    if (limits.size() != 6) {
        ROS_WARN_STREAM("The number of elements in limits is '" << limits.size() << "' but should be '" << 6 << "'");
        return false;
    }

    _ur.forceModeStart(refToffset, selection, wrenchTarget, limits);
    /* There is no (immediate) feedback from the _ur.forceModeStart() function call, so just returning true. */
    return true;
}

bool UniversalRobots::forceModeUpdate(const rw::math::Wrench6D<>& wrenchTarget)
{
    ROS_DEBUG_STREAM("New wrench target for forceModeUpdate: " << wrenchTarget);

    _ur.forceModeUpdate(wrenchTarget);
    return true;
}

bool UniversalRobots::forceModeStop()
{
    _ur.forceModeEnd();
    return true;
}

/************************************************************************
 * SerialDeviceServiceInterface
 ************************************************************************/
bool UniversalRobots::moveLin(const TransformAndSpeedContainer_t& targets)
{
    ROS_DEBUG_STREAM("moveLin with " << targets.size() << " target(s).");
    for (const auto& target : targets) {
        /* moveT(...) is void, so no errorcode is returned. Furthermore the implementation (at least in revision 5472) doesn't make use of the speeds. */
        _ur.moveT(std::get<0>(target), std::get<1>(target)); /* This could be rewritten to be more explicit about the parameters, such that std::get<1> is the speed */
    }

    /* Simply return true, given there is no feedback from the _ur.moveT(...) function */
    return true;
}

bool UniversalRobots::movePTP(const QAndSpeedContainer_t& targets)
{
    ROS_DEBUG_STREAM("movePTP with " << targets.size() << " target(s).");

    for (const auto& target : targets) {
        _ur.moveQ(std::get<0>(target), std::get<1>(target));
    }

    return true;
}

bool UniversalRobots::movePTP_T(const TransformAndSpeedContainer_t& targets)
{
/* FIXME:
 * [ If locking is necessary ] Replace the boost mutex scoped_lock with std::unique_lock (and the _mutex type, etc. to get rid of boost dependencies that are unnecessary). <- if locking should be done! (probably useful due to the _qcurrent and _state variables being global within this class...)
 *
 * Look at the implementation of moveLin - it seems to just use _ur.moveT(std::get<0>(target), std::get<1>(target)) compatible things - no need to convert the transform targets into q-space using an inverse kinematic solver... - Seems more efficient and easier...
 */
//    boost::mutex::scoped_lock lock(_mutex);
    
    for (const auto& target : targets) {
        _device->setQ(_qcurrent, _state);
        rw::math::Transform3D<> transform = std::get<0>(target);
        std::vector<rw::math::Q> solutions = _iksolver->solve(transform, _state);
        if (solutions.empty()) {
            ROS_WARN_STREAM("movePTP_T: Unable to find IK solution for: " << transform << " with qcurrent: " << _qcurrent);
            return false;
        }
        _ur.moveQ(solutions.front(), std::get<1>(target));
    }

    return true;
}

/* Forwarding the movement to the URServiceInterface function servoQ */
bool UniversalRobots::moveVelQ(const rw::math::Q& q_vel) {
    ROS_ERROR_STREAM("Current implementation does not follow the specification!");
    return false;

/************************************************************************
 * Old implementation
 ************************************************************************/
#if 0
    /* TODO:
     * Missing documentation on why the factor 0.1 is used and not some other arbitrary value?
     * And 1/10th of the value is added directly to the current joint values/angles, making a q-value of 0-100 (%) up to 10 radians, which is quite a lot - I doubt that this was the intension when it got implemented in MARVIN...
     */
    return servoQ(_qcurrent + q_vel*0.1);
#endif
}

/* Forwarding the movement to the URServiceInterface function servoQ */
bool UniversalRobots::moveVelT(const rw::math::VelocityScrew6D<>& t_vel)
{
    ROS_ERROR_STREAM("Current implementation has not been verified to follow the specification!");
    return false;

/************************************************************************
 * Old implementation
 ************************************************************************/
#if 0
    /* FIXME: Use c++11 mutex and unique_lock */
//	boost::mutex::scoped_lock lock(_mutex);
	_device->setQ(_qcurrent, _state);
	rw::math::Jacobian jac = _device->baseJend(_state);
        /* TODO:
         * Find out where the 'Eigen' got introduced from, since it doesn't exist...
         *	rw::math::Jacobian jacInv( rw::math::LinearAlgebra::pseudoInverseEigen(jac.e()) );
         */
	rw::math::Jacobian jacInv( rw::math::LinearAlgebra::pseudoInverse(jac.e()) );

	/* TODO:
         * Could use some more documentation on why the factor of 0.1 is being used (see todo comment for moveVelQ)
         */
        rw::math::Q qtarget = _qcurrent + (jacInv*t_vel)*0.1;
	return servoQ(qtarget);
#endif
}

/* Forwarding the movement to the URServiceInterface function servoT */
bool UniversalRobots::moveLinFC(const rw::math::Transform3D<>& posTarget, const rw::math::Transform3D<>& offset, const rw::math::Wrench6D<>& wrenchTarget, const rw::math::Q& controlGain)
{
    ROS_ERROR_STREAM("Current implementation has not been verified to follow the specification nor been tested in CAROS!");
    return false;

/************************************************************************
 * Implementation
 ************************************************************************/
#if 0
    if (_ftFrame == NULL) {
        ROS_WARN_STREAM("Unable to use force command without having defined a FT frame for the sensor.");
        return false;
    }

    rw::kinematics::State state = _workcell->getDefaultState();
    _device->setQ(_qcurrent, state);

    rw::math::Transform3D<> baseTref = rw::math::Transform3D<>::identity();
    rw::math::Transform3D<> baseTtarget = baseTref*posTarget;
    rw::math::Transform3D<> baseToffset = baseTref*offset;
    rw::math::Transform3D<> base2ft = _device->baseTframe(_ftFrame, state);
    rw::math::Transform3D<> baseTend = _device->baseTend(state);
    rw::math::Transform3D<> endTtarget = inverse(baseTend)*baseTtarget;
    rw::math::Transform3D<> endToffset = inverse(baseTend)*baseToffset;
    rw::math::Transform3D<> ftToffset = inverse(base2ft)*baseToffset;

    /* TODO:
     * If controlGain consist of elements that are all 0 then the _wrenchDataQueue element will not be used anyway and could be a case that should still work even when no or not enough wrench data is being published?
     *
     * FIXME: TODO:
     * Should look at the timestamps of the wrench data that has been received and verify that the received wrench data is within the allowed timestamp difference - not too old (or newer - should be caught in a ROS_WARN / test <- should this actually be an error or fatal error - maybe even an assert? as something really bogus could be going on...)
     * ^- Maybe it would make sense to function-locally(i.e. static) save the last used wrench timestamp to make sure that older samples in the queue are not being used (but then they will still remain there to be used for averaging/mean sampling.
     */
    if (_wrenchDataQueue.empty()) {
        ROS_WARN_STREAM("There have been no new wrench data coming from the ROS topic '" << _subFTData.getTopic() << "'");
        return false;
    }
    ROS_ASSERT(!_wrenchDataQueue.empty());
    /* Do not alter the _wrenchDataQueue - just make a copy of the newest element */
    rw::math::Wrench6D<> wrenchCurrent = _wrenchDataQueue.back();

    wrenchCurrent.setForce(ftToffset.R() * wrenchCurrent.force());
    wrenchCurrent.setTorque(ftToffset.R() * wrenchCurrent.torque());

    rw::math::Wrench6D<> wrenchDiff (wrenchTarget.force() - wrenchCurrent.force(), wrenchTarget.torque() - wrenchCurrent.torque());

    rw::math::Vector3D<> posOffset = endToffset.R() * endTtarget.P();
    rw::math::EAA<> eaaOffset = endToffset.R() * rw::math::EAA<>(endTtarget.R());

    /* Verify that the size of controlGain is 6 (i.e. the size of Wrench6D) */
    ROS_ASSERT(controlGain.size() == 6);
    for (unsigned int index = 0; index < 3; ++index) {
        posOffset(index) = controlGain(index) * wrenchDiff(index);
        eaaOffset(index) = controlGain(index+3) * wrenchDiff(index+3);
    }

    endToffset = rw::math::Transform3D<>(posOffset, eaaOffset);

    rw::math::Transform3D<> baseTtarget2 = baseTend * endToffset;

    return servoT(baseTtarget2);
#endif
}

bool UniversalRobots::moveServoQ(const QAndSpeedContainer_t& targets) {
    /* TODO: 
     * Throwing away the speed:
     * RobWorkHardware doesn't support specifying the speed when servoing (because the Universal Robot didn't or doesn't support user specified speeds when doing servoing.
     */
    bool res = false;
    for (const auto& target : targets) {
        res = servoQ(std::get<0>(target));
        if (!res) {
            break;
        }
    }

    return res;
}

bool UniversalRobots::moveServoT(const TransformAndSpeedContainer_t& targets) {
    bool res = false;
    /* Throwing away the speed */
    for (const auto& target : targets)
    {
        res = servoT(std::get<0>(target));
        if (!res)
        {
            break;
        }
    }

    return res;
}

bool UniversalRobots::moveStart() {
    ROS_ERROR_STREAM("Currently not implemented!");
    return false;

    /* TODO:
     * Does this make sense? - how to start the robot and what if it's already started?
     * ^- Isn't it already being started when launching this node?
     */
}

bool UniversalRobots::moveStop() {
    _ur.stopRobot();

    return true;
}

bool UniversalRobots::movePause() {
    ROS_ERROR_STREAM("Currently not implemented!");
    return false;
}

bool UniversalRobots::moveSetSafeModeEnabled(const bool value) {
    ROS_ERROR_STREAM("Currently not implemented!");
    return false;
}
