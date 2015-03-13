#include "ros/ros.h"
#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

//#include <caros_grasp_selection/SelectGrasps.hpp>
#include "GraspDBRepository.hpp"
#include <caros/common.h>

#include <caros_grasp_selection/SelectGrasps.h>

#include <rwlibs/task.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
//#include <rwlibs/algorithms/grasping/SelectGraspsPlanner.hpp>
#include <rw/models/RigidObject.hpp>
#include <rw/kinematics.hpp>
#include <rw/rw.hpp>

using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::common;
using namespace rw::math;
using namespace rwlibs::task;

bool selectGrasps(caros_grasp_selection::SelectGrasps::Request& req, caros_grasp_selection::SelectGrasps::Response& resp);
bool simpleSelectGrasps(caros_grasp_selection::SelectGrasps::Request& req, caros_grasp_selection::SelectGrasps::Response& resp);

// global variables
WorkCell::Ptr workcell;
Device::Ptr gripperDev;
Device::Ptr armDev;
GraspDBRepository::Ptr gdbRep;

// main initialization
int main(int argc, char **argv)
{
	// initialize ros
	ros::init(argc, argv, "GraspSelection");

	// initialize the node handle
    ros::NodeHandle nh("~");

    std::string gripperName, graspDBPath, stateTopicName, armName;
    // get properties on gripper name and on grasp database directory
    nh.getParam("gripper", gripperName);
    nh.getParam("arm", armName);
    nh.getParam("graspdb", graspDBPath);
    nh.getParam("state", stateTopicName); // topic that sends RobWork states

	// initialize workspace information, we need to know which gripper to use
	// and its kinematic/geometric model description

	try {
		workcell = caros::getWorkCell();
	} catch (const std::exception& exp) {
	    ROS_ERROR_STREAM("Error: "<<exp.what());
		return 1;
	}
	if(workcell==NULL){
		return 1;
	}

	// now get the gripper that we are supposed to use
	gripperDev = workcell->findDevice(gripperName);
	if(!gripperDev){
		std::stringstream sstr;
		BOOST_FOREACH(Device::Ptr dev, workcell->getDevices()){ sstr << dev->getName() << ", "; }
		ROS_ERROR_STREAM("No gripper with name "<< gripperName << " in workcell! Options are: " << sstr.str() );
		return 2;
	}

	// setup recieving of topics

	// next we set up the grasp database repository
	ROS_INFO_STREAM("construct");
	gdbRep = ownedPtr( new GraspDBRepository(graspDBPath) );
	ROS_INFO_STREAM("Initialize");
	gdbRep->initialize();
	ROS_INFO_STREAM("Initialized");
	// now setup the interface for calling the grasp selector
    ros::ServiceServer servglobal = nh.advertiseService<caros_grasp_selection::SelectGrasps::Request, caros_grasp_selection::SelectGrasps::Response>("select_grasps", simpleSelectGrasps);

    ros::spin();

	return 0;
}

bool selectGrasps(caros_grasp_selection::SelectGrasps::Request& req, caros_grasp_selection::SelectGrasps::Response& resp){
	// here comes the good stuff ...

	// if sensor image is there then take it and convert to collision geometry

	// get other relevant state information, (configuration of robot arm, of gripper, and such)

	// call the grasp planner to find a number of feasible grasps

	// SelectGraspsPlanner planner(wc, gripperDev, object, );
	return true;
}

Transform3D<> curr_wTbase;
Transform3D<> curr_wTobject;


bool wayToSortTargets(boost::tuple<GraspSubTask*,GraspTarget*,Transform3D<> > i, boost::tuple<GraspSubTask*,GraspTarget*, Transform3D<> > j) {
	// here we should use a scoring function
	// if a grasp has quality the we use that
	Vector3D<> curr_approach_i = curr_wTbase*i.get<2>()*Vector3D<>::z();
	Vector3D<> curr_approach_j = curr_wTbase*j.get<2>()*Vector3D<>::z();

	// currently we don't know about grasp quality so we just pick the one closest to current configuration
	return angle((curr_wTobject*i.get<1>()->pose) * Vector3D<>::z(), curr_approach_i) >
		angle((curr_wTobject*j.get<1>()->pose) * Vector3D<>::z(), curr_approach_j);
	//return i > j;
}

bool simpleSelectGrasps(caros_grasp_selection::SelectGrasps::Request& req, caros_grasp_selection::SelectGrasps::Response& resp){
	// here comes the good stuff ...
	std::string obj_id = req.object_id;
	int maxtime = std::max(req.max_time, 100);
	int maxgrasps = std::max(req.max_nr_grasps,10);

	// get other relevant state information, (configuration of robot arm, of gripper, and such)
	rw::kinematics::State::Ptr stateptr = caros::getState();

	rw::kinematics::State curr_state = *stateptr;
	rw::kinematics::State state = *stateptr;


	Transform3D<> pose = caros::toRw( req.object_pose );
	Transform3D<> wTobj_pose = pose; // we need to know which frame the object is represented in


	Object::Ptr object = workcell->findObject( obj_id );
	if(!object){
		std::stringstream sstr;
		BOOST_FOREACH(Object::Ptr dev, workcell->getObjects()){ sstr << dev->getName() << ", "; }
		ROS_WARN_STREAM("No object with name "<< obj_id << " in workcell! Options are: " << sstr.str()
				<< ". \n Hence, object geometry will not be used in collision detection!");
	} else {
		// move object to the estimated pose
		wTobj_pose = Kinematics::worldTframe(object->getBase(), state);
	}


	if( req.cloud_external.size()>0 ){
		rw::kinematics::Frame *sframe = NULL;
		if(req.sensor_frame_name==""){
			ROS_WARN_STREAM( "no sensor frame defined, using World frame." );
			sframe = workcell->getWorldFrame();
		} else {
			sframe = workcell->findFrame(req.sensor_frame_name);
			if( sframe==NULL ){
				ROS_WARN_STREAM( "sensor frame defined \"" << req.sensor_frame_name << "\", but not found in workcell description, using World frame." );
				sframe = workcell->getWorldFrame();
			}
		}
	}

	// get relevant grasp database
	std::vector<rwlibs::task::GraspTask::Ptr> grasps = gdbRep->getGrasps( gripperDev->getName() , obj_id, req.object_types );
	ROS_INFO_STREAM("Found " << grasps.size() << " databases for gripper!");

	// TODO: if sensor image is there then take it and convert it to collision geometry

	// don't count loading time
	rw::common::Timer rwtime;

	Transform3D<> initialBase = gripperDev->getBase()->getTransform(state);
	FixedFrame* fbase = dynamic_cast<FixedFrame*>(gripperDev->getBase());
	MovableFrame* mbase = dynamic_cast<MovableFrame*>(gripperDev->getBase());

	rw::proximity::CollisionDetector cdetect(workcell, ownedPtr( new rwlibs::proximitystrategies::ProximityStrategyPQP()) );

	// here we would call a fancy planner, instead we just iterate through all possible grasp targets,
	// sort them according to quality and return them
	std::vector<boost::tuple<GraspSubTask*,GraspTarget*,Transform3D<> > > colfree_targets;
	 rw::trajectory::TimedStatePath path;
	BOOST_FOREACH( GraspTask::Ptr gtask, grasps){
		std::vector<std::pair<GraspSubTask*,GraspTarget*> > targets = gtask->getAllTargets();
		std::cout << "targets: " << targets.size() << std::endl;
		std::string tcp_name = gtask->getTCPID();
		Frame *tcp = workcell->findFrame(tcp_name);
		if(!tcp){
			ROS_WARN_STREAM("Grasp task TCP is invalid! tcp: \"" << tcp_name << "\"");
			continue;
		}
		Transform3D<> baseTtcp = Kinematics::frameTframe(gripperDev->getBase(),tcp,curr_state);
		// test target for collision
		for(std::size_t i=0;i<targets.size();i++){
			//if(rwtime.currentTimeMs()>maxtime){
			//	break;
			//}

			Transform3D<> objTtcp = targets[i].second->pose;

			// place the gripper
			//if(object) object->getBase()
			if(mbase) mbase->moveTo(wTobj_pose * objTtcp * inverse(baseTtcp), workcell->getWorldFrame(), state);
			else if(fbase) fbase->moveTo( wTobj_pose*objTtcp * inverse(baseTtcp), workcell->getWorldFrame(), state );

			// set gripper fingers

			gripperDev->setQ( targets[i].first->openQ, state );
			rw::proximity::CollisionDetector::QueryResult res;
			// test collision

			if( cdetect.inCollision(state, &res, true) ){
				//std::cout << "Colliding frames:\n";
				//BOOST_FOREACH(FramePair fpair, res.collidingFrames){
				//	std::cout << fpair.first->getName() << " -- " << fpair.second->getName() << std::endl;
				//}
				continue;
			}

			path.push_back( rw::trajectory::TimedState(i,state) );
			// next test if we can do inverse kinematics to the pose


			// else add grasp target to solution space
			colfree_targets.push_back(boost::make_tuple(targets[i].first,targets[i].second, baseTtcp) );

		}
		//if(rwtime.currentTimeMs()>maxtime){
		//	break;
		//}
	}
	rw::loaders::PathLoader::storeTimedStatePath(*workcell,path,"collisionfree.rwplay");

	// now return the best 100 grasps
	ROS_INFO_STREAM("Found " << colfree_targets.size() << " collision free targets!");
	ROS_INFO_STREAM("Looking for best " << maxgrasps << "targets!");

	curr_wTbase = Kinematics::worldTframe( gripperDev->getBase(), curr_state );
	curr_wTobject = wTobj_pose;//Kinematics::worldTframe( object->getBase(), curr_state );
	std::sort(colfree_targets.begin(), colfree_targets.end(), wayToSortTargets);

	// now copy grasp targets into ros format
	for(std::size_t i=0;i<std::min((size_t)maxgrasps,colfree_targets.size());i++){
		GraspSubTask* stask = colfree_targets[i].get<0>();
		rwlibs::task::GraspTarget* ttask = colfree_targets[i].get<1>();
		caros_grasp_selection::GraspTarget target;
		target.gripper_name = gripperDev->getName();
		target.q_approach = stask->openQ.toStdVector();
		target.q_grasp = stask->closeQ.toStdVector();
		target.q_tau = stask->tauMax.toStdVector();
		target.tcp_frame_name = gripperDev->getBase()->getName();
                target.tcp_pose = caros::toRosPose( wTobj_pose * ttask->pose * inverse(colfree_targets[i].get<2>()));
		resp.targets.push_back( target );
	}

	return true;
}
