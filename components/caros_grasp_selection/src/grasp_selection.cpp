#include "ros/ros.h"
#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

//#include <caros_grasp_selection/SelectGrasps.hpp>
#include "GraspDBRepository.hpp"
#include <caros/common_robwork.h>
#include <sensor_msgs/PointCloud2.h>

#include <caros_grasp_selection/SelectGrasps.h>

#include "pcl_ros/point_cloud.h"

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

bool selectGrasps(caros_grasp_selection::SelectGrasps::Request& req,
                  caros_grasp_selection::SelectGrasps::Response& resp);
bool simpleSelectGrasps(caros_grasp_selection::SelectGrasps::Request& req,
                        caros_grasp_selection::SelectGrasps::Response& resp);

// global variables
WorkCell::Ptr workcell;
Device::Ptr gripperDev;
SerialDevice::Ptr armDev;
Object::Ptr object;
GraspDBRepository::Ptr gdbRep;

Transform3D<> curr_refTbase;
Transform3D<> curr_refTobject;

rw::kinematics::State curr_state;
rw::kinematics::State state;

FixedFrame* fbase=NULL;
MovableFrame* mbase=NULL;

rw::trajectory::TimedStatePath pathDebug;



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

  try
  {
    workcell = caros::getWorkCell();
  }
  catch (const std::exception& exp)
  {
    ROS_ERROR_STREAM("Error: "<<exp.what());
    return 1;
  }
  if (workcell == NULL)
  {
    return 1;
  }

  // now get the gripper that we are supposed to use
  gripperDev = workcell->findDevice(gripperName);
  if (!gripperDev)
  {
    std::stringstream sstr;
    BOOST_FOREACH(Device::Ptr dev, workcell->getDevices())
    {
      sstr << dev->getName() << ", ";
    }
    ROS_ERROR_STREAM("No gripper with name "<< gripperName << " in workcell! Options are: " << sstr.str());
    return 2;
  }

  fbase = dynamic_cast<FixedFrame*>(gripperDev->getBase());
  mbase = dynamic_cast<MovableFrame*>(gripperDev->getBase());


  // now get the gripper that we are supposed to use
  armDev = workcell->findDevice<SerialDevice>(armName);
  if (!armDev)
  {
    std::stringstream sstr;
    BOOST_FOREACH(Device::Ptr dev, workcell->getDevices())
    {
      if( dynamic_cast<SerialDevice*>(dev.get()) )
        sstr << dev->getName() << ", ";
    }
    ROS_WARN_STREAM("No serialdevice with name "<< armDev << " in workcell! Options are: " << sstr.str());
  }


  // setup recieving of topics

  // next we set up the grasp database repository
  ROS_INFO_STREAM("construct");
  gdbRep = ownedPtr(new GraspDBRepository(graspDBPath));
  ROS_INFO_STREAM("Initialize");
  gdbRep->initialize();
  ROS_INFO_STREAM("Initialized");
  // now setup the interface for calling the grasp selector
  ros::ServiceServer servglobal_simple = nh.advertiseService<caros_grasp_selection::SelectGrasps::Request,
      caros_grasp_selection::SelectGrasps::Response>("select_grasps_simple", simpleSelectGrasps);

  ros::ServiceServer servglobal = nh.advertiseService<caros_grasp_selection::SelectGrasps::Request,
      caros_grasp_selection::SelectGrasps::Response>("select_grasps", selectGrasps);

  ros::spin();

  return 0;
}


bool wayToSortTargets(boost::tuple<GraspSubTask*, GraspTarget*, Transform3D<> > i,
                      boost::tuple<GraspSubTask*, GraspTarget*, Transform3D<> > j)
{
  // here we should use a scoring function
  // if a grasp has quality the we use that
  Vector3D<> curr_approach_i = curr_refTbase * i.get<2>() * Vector3D<>::z();
  Vector3D<> curr_approach_j = curr_refTbase * j.get<2>() * Vector3D<>::z();

  // currently we don't know about grasp quality so we just pick the one closest to current configuration
  return angle((curr_refTobject * i.get<1>()->pose) * Vector3D<>::z(), curr_approach_i)
      > angle((curr_refTobject * j.get<1>()->pose) * Vector3D<>::z(), curr_approach_j);
  //return i > j;
}


std::vector<boost::tuple<GraspSubTask*, GraspTarget*, Transform3D<> > >
  filterCollision(
                       Frame *refFrame,
                       Transform3D<> refTobj_pose,
                       rw::proximity::CollisionDetector& detector,
                       Frame *tcp,
                       std::vector<std::pair<GraspSubTask*, GraspTarget*> >& grasps){
  std::vector<boost::tuple<GraspSubTask*, GraspTarget*, Transform3D<> > > filteredGrasps;

  Transform3D<> baseTtcp = Kinematics::frameTframe(gripperDev->getBase(), tcp, curr_state);
  // test target for collision
  for (int i = 0; i < grasps.size(); i++)
  {
    Transform3D<> objTtcp = grasps[i].second->pose;

    // place the gripper
    if (mbase){
      mbase->moveTo(refTobj_pose * objTtcp * inverse(baseTtcp), refFrame, state);
    } else if (fbase) {
      fbase->moveTo(refTobj_pose * objTtcp * inverse(baseTtcp), refFrame, state);
    }
    // set gripper fingers

    gripperDev->setQ(grasps[i].first->openQ, state);
    rw::proximity::CollisionDetector::QueryResult res;
    // test collision
    if (detector.inCollision(state, &res, true))
    {
      //std::cout << "Colliding frames:\n";
      //BOOST_FOREACH(FramePair fpair, res.collidingFrames){
      //      std::cout << fpair.first->getName() << " -- " << fpair.second->getName() << std::endl;
      //}
      continue;
    }
    // only for debugging
    pathDebug.push_back(rw::trajectory::TimedState(i, state));

    // add grasp target to solution space
    filteredGrasps.push_back(boost::make_tuple(grasps[i].first, grasps[i].second, baseTtcp));
  }

  return filteredGrasps;
}


std::vector<boost::tuple<GraspSubTask*, GraspTarget*, Transform3D<> > >
  filterCloudCollision(std::vector<sensor_msgs::PointCloud2>& rgbcloudList,
                       Frame *refFrame,
                       Transform3D<> refTobj_pose,
                       rw::proximity::CollisionDetector& detector,
                       Frame *tcp,
                       std::vector<std::pair<GraspSubTask*, GraspTarget*> >& grasps){
  std::vector<boost::tuple<GraspSubTask*, GraspTarget*, Transform3D<> > > filteredGrasps;

  // add pointcloud to collision detector
  filteredGrasps = filterCollision(refFrame,refTobj_pose,detector,tcp,grasps);
  // remove pointcloud from collision detector

  return filteredGrasps;
}

std::vector<boost::tuple<GraspSubTask*, GraspTarget*, Transform3D<> > >
  filterIK(SerialDevice::Ptr robot,
                       Frame *refFrame,
                       Transform3D<> refTobj_pose,
                       rw::proximity::CollisionDetector& detector,
                       Frame *tcp,
                       std::vector<boost::tuple<GraspSubTask*, GraspTarget*, Transform3D<> > >& grasps){
  std::vector<boost::tuple<GraspSubTask*, GraspTarget*, Transform3D<> > > filteredGrasps = grasps;


  return filteredGrasps;
}

std::vector<boost::tuple<GraspSubTask*, GraspTarget*, Transform3D<> > >
  sortWithIK(SerialDevice::Ptr robot,
                       Frame *refFrame,
                       Transform3D<> refTobj_pose,
                       Frame *tcp,
                       std::vector<boost::tuple<GraspSubTask*, GraspTarget*, Transform3D<> > >& grasps)
{
  std::vector<boost::tuple<GraspSubTask*, GraspTarget*, Transform3D<> > > filteredGrasps = grasps;


  return filteredGrasps;
}

std::vector<boost::tuple<GraspSubTask*, GraspTarget*, Transform3D<> > >
  sortSimple(
                       Frame *refFrame,
                       Transform3D<> refTobj_pose,
                       Frame *tcp,
                       std::vector<boost::tuple<GraspSubTask*, GraspTarget*, Transform3D<> > >& grasps){
  std::vector<boost::tuple<GraspSubTask*, GraspTarget*, Transform3D<> > > filteredGrasps = grasps;


  return filteredGrasps;
}

bool selectGrasps(caros_grasp_selection::SelectGrasps::Request& req,
                  caros_grasp_selection::SelectGrasps::Response& resp)
{
  // here comes the good stuff ...
  pathDebug.clear();
  std::string obj_id = req.object_id;
  int maxtime = std::max(req.max_time, 100);
  int maxgrasps = std::max(req.max_nr_grasps, 10);

  // get other relevant state information, (configuration of robot arm, of gripper, and such)
  rw::kinematics::State::Ptr stateptr = caros::getState();

  curr_state = *stateptr;
  state = *stateptr;

  Transform3D<> pose = caros::toRw(req.object_pose);
  Transform3D<> refTobj_pose = pose; // we need to know which frame the object is represented in

  std::string refFrameName = req.sensor_frame_name;


  Frame* refFrame = workcell->findFrame(refFrameName);
  if (!refFrame){
    std::stringstream sstr;
    BOOST_FOREACH(Frame* f, workcell->getFrames()){sstr << f->getName() << ", ";}
    ROS_ERROR_STREAM(
        "No frame with name "<< refFrameName << " in workcell! Options are: " << sstr.str()
        << ". \n A reference frame of pose and pointcloud data is REQUIRED!");
    refFrame = workcell->getWorldFrame();
  }


  object = workcell->findObject(obj_id);
  if (!object)
  {
    std::stringstream sstr;
    BOOST_FOREACH(Object::Ptr dev, workcell->getObjects())
    {
      sstr << dev->getName() << ", ";
    }
    ROS_WARN_STREAM(
        "No object with name "<< obj_id << " in workcell! Options are: " << sstr.str() << ". \n Hence, object geometry will not be used in collision detection!");
  }
  else
  {
    // move object to the estimated pose
    //refTobj_pose = Kinematics::frameTframe(refFrame, object->getBase(), state);
  }


  // get relevant grasp database
  std::vector<rwlibs::task::GraspTask::Ptr> grasps = gdbRep->getGrasps(gripperDev->getName(), obj_id, req.object_types);
  ROS_INFO_STREAM("Found " << grasps.size() << " databases for gripper!");

  // TODO: if sensor image is there then take it and convert it to collision geometry

  // don't count loading time
  rw::common::Timer rwtime;

  Transform3D<> initialBase = gripperDev->getBase()->getTransform(state);


  rw::proximity::CollisionDetector cdetect(workcell, ownedPtr(new rwlibs::proximitystrategies::ProximityStrategyPQP()));

  // here we would call a fancy planner, instead we just iterate through all possible grasp targets,
  // sort them according to quality and return them
  std::vector<boost::tuple<GraspSubTask*, GraspTarget*, Transform3D<> > > colfree_targets;


  BOOST_FOREACH( GraspTask::Ptr gtask, grasps)
  {
    std::string tcp_name = gtask->getTCPID();
    Frame *tcp = workcell->findFrame(tcp_name);
    if (!tcp)
    {
      ROS_WARN_STREAM("Grasp task TCP is invalid! tcp: \"" << tcp_name << "\"");
      continue;
    }

    std::vector<std::pair<GraspSubTask*, GraspTarget*> > targets = gtask->getAllTargets();

    // remove any grasps that make gripper collide with point cloud
    // this require: pointcloud, sensorframe, object pose,
    if( req.cloud_external.size() > 0 && refFrame!=workcell->getWorldFrame() ){
      colfree_targets = filterCloudCollision(req.cloud_external, refFrame, refTobj_pose, cdetect, tcp, targets);
    }
    ROS_INFO_STREAM("Grasps after collision filtering" << targets.size());

    // remove any grasps that cannot be reached by the robot arm
    // this require:  serialdevice, refFrame, object pose
    if( armDev !=NULL ){
      colfree_targets = filterIK( armDev, refFrame, refTobj_pose, cdetect, tcp, colfree_targets);
    }
    ROS_INFO_STREAM("Grasps after IK filtering" << targets.size());

    // sort the targets according to quality,
    if( armDev!=NULL ){
      colfree_targets = sortWithIK(armDev, refFrame, refTobj_pose, tcp, colfree_targets);
    } else {
      // fallback to simple quality sorting
      colfree_targets = sortSimple(refFrame, refTobj_pose, tcp, colfree_targets);
    }
  }

  rw::loaders::PathLoader::storeTimedStatePath(*workcell, pathDebug, "collisionfree.rwplay");

  // now return the best 100 grasps
  ROS_INFO_STREAM("Found " << colfree_targets.size() << " collision free targets!");
  ROS_INFO_STREAM("Looking for best " << maxgrasps << "targets!");

  curr_refTbase = Kinematics::frameTframe(refFrame, gripperDev->getBase(), curr_state);
  curr_refTobject = refTobj_pose;           //Kinematics::worldTframe( object->getBase(), curr_state );
  std::sort(colfree_targets.begin(), colfree_targets.end(), wayToSortTargets);

  // now copy grasp targets into ros format
  for (int i = 0; i < std::min((size_t)maxgrasps, colfree_targets.size()); i++)
  {
    GraspSubTask* stask = colfree_targets[i].get<0>();
    rwlibs::task::GraspTarget* ttask = colfree_targets[i].get<1>();
    caros_grasp_selection::GraspTarget target;
    target.gripper_name = gripperDev->getName();
    target.q_approach = stask->openQ.toStdVector();
    target.q_grasp = stask->closeQ.toStdVector();
    target.q_tau = stask->tauMax.toStdVector();
    target.tcp_frame_name = gripperDev->getBase()->getName();
    target.tcp_pose = caros::toRosPose(refTobj_pose * ttask->pose * inverse(colfree_targets[i].get<2>()));
    resp.targets.push_back(target);
  }

  return true;
}


namespace {
/*
  rw::geometry::PointCloud::Ptr toRw(const sensor_msgs::PointCloud2& rgbcloud){
    int w = rgbcloud.width;
    int h = rgbcloud.height;
    rw::geometry::PointCloud::Ptr cloud = ownedPtr( new rw::geometry::PointCloud(w,h));
    pcl::PointCloud<pcl::PointXYZ> result;
    pcl::fromROSMsg(rgbcloud, result);
    for(int x=0;x<w;x++){
      for(int y=0;y<w;y++){


      }
    }
  }
  */
}















bool simpleSelectGrasps(caros_grasp_selection::SelectGrasps::Request& req,
                        caros_grasp_selection::SelectGrasps::Response& resp)
{
  // here comes the good stuff ...
  std::string obj_id = req.object_id;
  int maxtime = std::max(req.max_time, 100);
  int maxgrasps = std::max(req.max_nr_grasps, 10);

  // get other relevant state information, (configuration of robot arm, of gripper, and such)
  rw::kinematics::State::Ptr stateptr = caros::getState();

  rw::kinematics::State curr_state = *stateptr;
  rw::kinematics::State state = *stateptr;

  Transform3D<> pose = caros::toRw(req.object_pose);
  Transform3D<> wTobj_pose = pose; // we need to know which frame the object is represented in

  Object::Ptr object = workcell->findObject(obj_id);
  if (!object)
  {
    std::stringstream sstr;
    BOOST_FOREACH(Object::Ptr dev, workcell->getObjects())
    {
      sstr << dev->getName() << ", ";
    }
    ROS_WARN_STREAM(
        "No object with name "<< obj_id << " in workcell! Options are: " << sstr.str() << ". \n Hence, object geometry will not be used in collision detection!");
  }
  else
  {
    // move object to the estimated pose
    wTobj_pose = Kinematics::worldTframe(object->getBase(), state);
  }

  if (req.cloud_external.size() > 0)
  {
    rw::kinematics::Frame *sframe = NULL;
    if (req.sensor_frame_name == "")
    {
      ROS_WARN_STREAM("no sensor frame defined, using World frame.");
      sframe = workcell->getWorldFrame();
    }
    else
    {
      sframe = workcell->findFrame(req.sensor_frame_name);
      if (sframe == NULL)
      {
        ROS_WARN_STREAM(
            "sensor frame defined \"" << req.sensor_frame_name << "\", but not found in workcell description, using World frame.");
        sframe = workcell->getWorldFrame();
      }
    }
  }


  // get relevant grasp database
  std::vector<rwlibs::task::GraspTask::Ptr> grasps = gdbRep->getGrasps(gripperDev->getName(), obj_id, req.object_types);
  ROS_INFO_STREAM("Found " << grasps.size() << " databases for gripper/object!");

  // don't count loading time
  rw::common::Timer rwtime;

  // TODO: if sensor image is there then take it and convert it to collision geometry






  Transform3D<> initialBase = gripperDev->getBase()->getTransform(state);
  FixedFrame* fbase = dynamic_cast<FixedFrame*>(gripperDev->getBase());
  MovableFrame* mbase = dynamic_cast<MovableFrame*>(gripperDev->getBase());

  rw::proximity::CollisionDetector cdetect(workcell, ownedPtr(new rwlibs::proximitystrategies::ProximityStrategyPQP()));

  // here we would call a fancy planner, instead we just iterate through all possible grasp targets,
  // sort them according to quality and return them
  std::vector<boost::tuple<GraspSubTask*, GraspTarget*, Transform3D<> > > colfree_targets;
  rw::trajectory::TimedStatePath path;
  BOOST_FOREACH( GraspTask::Ptr gtask, grasps)
  {
    std::vector<std::pair<GraspSubTask*, GraspTarget*> > targets = gtask->getAllTargets();
    std::cout << "targets: " << targets.size() << std::endl;
    std::string tcp_name = gtask->getTCPID();
    Frame *tcp = workcell->findFrame(tcp_name);
    if (!tcp)
    {
      ROS_WARN_STREAM("Grasp task TCP is invalid! tcp: \"" << tcp_name << "\"");
      continue;
    }
    Transform3D<> baseTtcp = Kinematics::frameTframe(gripperDev->getBase(), tcp, curr_state);
    // test target for collision
    for (int i = 0; i < targets.size(); i++)
    {
      //if(rwtime.currentTimeMs()>maxtime){
      //	break;
      //}

      Transform3D<> objTtcp = targets[i].second->pose;

      // place the gripper
      //if(object) object->getBase()
      if (mbase)
        mbase->moveTo(wTobj_pose * objTtcp * inverse(baseTtcp), workcell->getWorldFrame(), state);
      else if (fbase)
        fbase->moveTo(wTobj_pose * objTtcp * inverse(baseTtcp), workcell->getWorldFrame(), state);

      // set gripper fingers

      gripperDev->setQ(targets[i].first->openQ, state);
      rw::proximity::CollisionDetector::QueryResult res;
      // test collision

      if (cdetect.inCollision(state, &res, true))
      {
        //std::cout << "Colliding frames:\n";
        //BOOST_FOREACH(FramePair fpair, res.collidingFrames){
        //	std::cout << fpair.first->getName() << " -- " << fpair.second->getName() << std::endl;
        //}
        continue;
      }

      path.push_back(rw::trajectory::TimedState(i, state));
      // next test if we can do inverse kinematics to the pose

      // else add grasp target to solution space
      colfree_targets.push_back(boost::make_tuple(targets[i].first, targets[i].second, baseTtcp));

    }
    //if(rwtime.currentTimeMs()>maxtime){
    //	break;
    //}
  }
  rw::loaders::PathLoader::storeTimedStatePath(*workcell, path, "collisionfree.rwplay");

  // now return the best 100 grasps
  ROS_INFO_STREAM("Found " << colfree_targets.size() << " collision free targets!");
  ROS_INFO_STREAM("Looking for best " << maxgrasps << "targets!");

  curr_refTbase = Kinematics::worldTframe(gripperDev->getBase(), curr_state);
  curr_refTobject = wTobj_pose;		//Kinematics::worldTframe( object->getBase(), curr_state );
  std::sort(colfree_targets.begin(), colfree_targets.end(), wayToSortTargets);

  // now copy grasp targets into ros format
  for (int i = 0; i < std::min((size_t)maxgrasps, colfree_targets.size()); i++)
  {
    GraspSubTask* stask = colfree_targets[i].get<0>();
    rwlibs::task::GraspTarget* ttask = colfree_targets[i].get<1>();
    caros_grasp_selection::GraspTarget target;
    target.gripper_name = gripperDev->getName();
    target.q_approach = stask->openQ.toStdVector();
    target.q_grasp = stask->closeQ.toStdVector();
    target.q_tau = stask->tauMax.toStdVector();
    target.tcp_frame_name = gripperDev->getBase()->getName();
    target.tcp_pose = caros::toRosPose(wTobj_pose * ttask->pose * inverse(colfree_targets[i].get<2>()));
    resp.targets.push_back(target);
  }

  return true;
}
