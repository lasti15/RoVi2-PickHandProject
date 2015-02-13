#include <caros/SerialDeviceSIProxy.hpp>
#include <caros/common.h>

#include <rw/math/Q.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/pathplanning/QEdgeConstraint.hpp>
#include <rw/pathplanning/QConstraint.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>

#include <ros/ros.h>

#include <string>
#include <vector>
#include <cassert>

bool doCollisionDetectionPrototype(const rw::math::Q &startConfiguration, const rw::math::Q &endConfiguration) {
    ros::NodeHandle _nodehandle("~");

    rw::models::Device::Ptr _device;
    rw::kinematics::State _state;

    rw::models::WorkCell::Ptr _workcell;
    _workcell = caros::getWorkCell();
    if (_workcell == NULL) {
        ROS_ERROR("No workcell was loaded - exiting...");
        return false;
    }

    std::string deviceName;
    if (! _nodehandle.getParam("deviceName", deviceName)) {
        ROS_FATAL_STREAM("The parameter '" << _nodehandle.getNamespace() << "/deviceName' was not present on the parameter server! This parameter has to be specified for this test-node to work properly.");
        return false;
    }

    if (_workcell == NULL) {
        ROS_FATAL_STREAM("No workcell was provided!");
        return false;
    }

    ROS_ASSERT(_workcell != NULL);
    ROS_DEBUG_STREAM("Looking for the device '" << deviceName << "' in the workcell.");
    _device = _workcell->findDevice(deviceName);
    if (_device == NULL) {
        ROS_FATAL_STREAM("Unable to find device " << deviceName << " in the loaded workcell");
        return false;
    }

    _state = _workcell->getDefaultState();

    ROS_DEBUG_STREAM("Doing path collision checking...");
    /* TODO:
     * [ IMPORTANT ] Has to know the state of the other UR (UR2 also) in its live configuration, and not the configuration that was hardcoded within the scene/workcell.
     */

    auto detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_workcell, rwlibs::proximitystrategies::ProximityStrategyYaobi::make()));
    const rw::pathplanning::PlannerConstraint plannerConstraint = rw::pathplanning::PlannerConstraint::make(detector, _device, _state);

    // start configuration
    if (plannerConstraint.getQConstraint().inCollision(startConfiguration)) {
        ROS_DEBUG_STREAM("The start configuration '" << startConfiguration << "' is in collision!");
        return false;
    }
    // end configuration
    if (plannerConstraint.getQConstraint().inCollision(endConfiguration)) {
        ROS_DEBUG_STREAM("The end configuration '" << endConfiguration << "' is in collision!");
        return false;
    }
    // path
    if (plannerConstraint.getQEdgeConstraint().inCollision(startConfiguration, endConfiguration)) {
        ROS_DEBUG_STREAM("The path between the start configuration '" << startConfiguration << "' and the end configuration '" << endConfiguration << "' is in collision!");
        return false;
    }

    ROS_DEBUG_STREAM("The path appears to be collision free.");

    return true;
}

bool doCollisionFreeServoing(caros::SerialDeviceSIProxy &sdsip, const rw::math::Q &startConfiguration, const rw::math::Q &endConfiguration) {
    bool status = false;
    bool collision_free_path = false;
    collision_free_path = doCollisionDetectionPrototype(startConfiguration, endConfiguration);

    if (collision_free_path) {
        ROS_INFO_STREAM("The path is collision free!");
        ROS_DEBUG_STREAM("Going to servo to: " << endConfiguration);
        status = sdsip.moveServoQ(endConfiguration);
        ROS_INFO_STREAM("moveServoQ returned: " << status);
    } else {
        ROS_INFO_STREAM("The path was NOT collision free!");
    }

    return status;
}

int testUsingMoveServoQ(void) {
    int retValue = 0;
    ros::NodeHandle nh("~");
    caros::SerialDeviceSIProxy sdsip(nh, "ur_simple_demo_node");
    double qChange = 0;
    int steps = 0;
    int movements = 0;

    if (! nh.getParam("qChange", qChange)) {
        ROS_FATAL_STREAM("The parameter '" << nh.getNamespace() << "/qChange' was not present on the parameter server! This parameter has to be specified for this test-node to work properly.");
        return -2;
    }

    if (! nh.getParam("steps", steps)) {
        ROS_FATAL_STREAM("The parameter '" << nh.getNamespace() << "/steps' was not present on the parameter server! This parameter has to be specified for this test-node to work properly.");
        return -2;
    }

    if (! nh.getParam("movements", movements)) {
        ROS_FATAL_STREAM("The parameter '" << nh.getNamespace() << "/movements' was not present on the parameter server! This parameter has to be specified for this test-node to work properly.");
        return -2;
    }

    ros::Time obtainedTimestamp = sdsip.getTimeStamp();
    ROS_DEBUG_STREAM("Timestamp right after initialisation: " << obtainedTimestamp);
    ROS_DEBUG_STREAM("Timestamp time is valid: " << obtainedTimestamp.isValid());

    /* Make sure to get and operate on fresh data from the serial device
     * It's assumed that the serial device is not moving
     * ^- That could be asserted / verified using sdsip.isMoving()
     * However other sources could invoke services on the UR that causes it to move...
     */
    ros::Time currentTimestamp = ros::Time::now();
    while (currentTimestamp > obtainedTimestamp) {
        ros::Duration(0.1).sleep(); // In seconds
        ros::spinOnce();
        obtainedTimestamp = sdsip.getTimeStamp();
    }

    /* TODO:
     * Can be rewritten to be more clear and use iterators and reverse iterators, etc?
     */
    /* steps = 0 no movement will happen
     * steps = 1 one movement will happen from startConfiguration to the specified change */
    for (int movementsCount = 0; movementsCount < movements; ++movementsCount) {
        rw::math::Q startConfiguration = sdsip.getQ();
        ROS_DEBUG_STREAM("The startConfiguration was found to be: " << startConfiguration);

        std::vector<rw::math::Q> waypoints;
        double stepSize = qChange;
        if (steps > 1) {
            stepSize = qChange / static_cast<double>(steps);
        }
        rw::math::Q qStepChange = rw::math::Q(startConfiguration.size(), stepSize);
        waypoints.push_back(startConfiguration);
        /* steps == 0, results in only the startconfiguration to be considered as a waypoint */
        for (int step = 1; step <= steps; ++step) {
            waypoints.push_back(startConfiguration + (step * qStepChange));
        }
        ROS_DEBUG_STREAM("The number of waypoints is: " << waypoints.size());
        /* The front() and back() member functions will cause undefined behaviour if invoked on an empty vector */
        assert(!waypoints.empty());
        ROS_ASSERT(!waypoints.empty());
        ROS_DEBUG_STREAM("The startConfiguration is specified to be: " << waypoints.front());
        ROS_DEBUG_STREAM("The endConfiguration is specified to be: " << waypoints.back());
        assert(waypoints.size() == steps + 1);
        ROS_ASSERT(waypoints.size() == steps + 1);

        for (unsigned int index = 1; index < waypoints.size(); ++index) {
            if (!doCollisionFreeServoing(sdsip, waypoints.at(index-1), waypoints.at(index))) {
                return -3;
            }
//            ros::Duration(0.01).sleep(); // In seconds
        }

        /* The construction of this loop requires index to be signed */
        for (int index = waypoints.size() - 2; index >= 0; --index) {
            if (!doCollisionFreeServoing(sdsip, waypoints.at(index+1), waypoints.at(index))) {
                return -3;
            }
//            ros::Duration(0.01).sleep(); // In seconds
        }
    }
    /* FIXME:
     * Idle until the movement has finished - look at the different flags/changes on the node - can it be seen when the node is going to move and then when it's finished moving
     * ^- The hard part seems to be the initial wait from sending the command until the receiving node has processed it and changed its state...
     *
     * A subscriber would also need ROS spinning to have the message queue/callback be processed...
     * ^- Actionlib has solved these issues (supposedly)
     */

    ROS_DEBUG_STREAM("Going to sleep for a short while...");
    ros::Duration(5).sleep(); // In seconds
    ROS_DEBUG_STREAM("Waking up after the short sleep.");

    ros::spinOnce();
    ROS_DEBUG_STREAM("The current q-configuration is: " << sdsip.getQ());

    return retValue;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "ur_simple_demo");

    return testUsingMoveServoQ();
}
