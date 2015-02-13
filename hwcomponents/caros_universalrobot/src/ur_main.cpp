/* TODO:
 * Properly document the parameters that this node supports/uses.
 */
#include <caros/UniversalRobots.hpp>

#include <caros/common.h>

#include <rw/loaders/WorkCellLoader.hpp>

#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "universalrobots");

    ros::NodeHandle nh("~");

    /* TODO:
     * This workcell setup can also be moved into the UniversalRobots.cpp class. Would that make more sense?
     */
    rw::models::WorkCell::Ptr workCell;
    workCell = caros::getWorkCell();
    if (workCell == NULL) {
        ROS_ERROR("No workcell was loaded - exiting...");
        return 1;
    }

    ROS_DEBUG("Workcell loaded - starting node.");

    caros::UniversalRobots ur(nh, workCell);
    ur.start();

    return 0;
}
