/**/
#ifndef CAROS_SIMPLEGRASPSELECTOR_HPP
#define CAROS_SIMPLEGRASPSELECTOR_HPP

#include <iostream>

#include <sstream>
#include <fstream>

#include <rw/models/Device.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/models/WorkCell.hpp>
#include <rwlibs/task/GraspTask.hpp>

#include <sandbox/learnbip/Planning.hpp>
#include <sandbox/learnbip/GraspDB.hpp>
#include <sandbox/learnbip/RandomSampler.hpp>
#include <sandbox/learnbip/Experiments.hpp>

/**
 * @brief this takes a workcell description and multiple grasp databases and enables
 * queries to how and where to grasp an object in the scene.
 */
class SimpleGraspSelector
{
public:

    SimpleGraspSelector(double looprate, std::string nodename);

    virtual ~SimpleGraspSelector();

    bool run();

    // enable setting the current configuration of the robot
    //void setRobotConfiguration();

    // compute a grasp configuration


protected:
    ros::NodeHandle _nodeHnd;
};

#endif //#ifndef OBJECTGRASPOACNEW_HPP
