/*
 * trackstar.cpp
 *
 *  Created on: Jan 28, 2013
 *      Author: mtt
 */

#include <caros/trakstar_node.h>

#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>

#include <rw/math.hpp>

#include "Trakstar.hpp"

using namespace rw::math;
using namespace caros;
//using namespace marvin_common;
// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_EXPORT_CLASS(caros::TrakstarNode, nodelet::Nodelet)

TrakstarNode::TrakstarNode():Nodelet(),
CarosNodeServiceInterface(ros::NodeHandle("~"), 240),
ButtonSensorServiceInterface(ros::NodeHandle("~")), PoseSensorServiceInterface(
    ros::NodeHandle("~")), nodehandle_(ros::NodeHandle("~"))

{
  t_driver_ = new caros::Trakstar();
}

TrakstarNode::TrakstarNode(const ros::NodeHandle& nodehandle) :
    CarosNodeServiceInterface(nodehandle, 240), ButtonSensorServiceInterface(nodehandle), PoseSensorServiceInterface(
        nodehandle), nodehandle_(nodehandle)
{
  // create a new Trakstar driver

  t_driver_ = new caros::Trakstar();
  /* Currently nothing specific should happen */
  //_stop = false;
  //_isrunning = false;
}

TrakstarNode::~TrakstarNode()
{
  //
}

void TrakstarNode::onInit(){
  pubThread_.reset(new boost::thread(boost::bind(&caros::TrakstarNode::start, this)));
  while(getState()==PREINIT){ros::spinOnce();};
  // now wait until node is finished initializing

}

bool TrakstarNode::activateHook()
{

  /************************************************************************
   * Parameters
   ************************************************************************/
  //if (! nodehandle_.getParam("device_name", deviceName)) {
  //    CAROS_FATALERROR("The parameter '" << nodehandle_.getNamespace() << "/device_name' was not present on the parameter server! This parameter has to be specified for this node to work properly.", URNODE_MISSING_PARAMETER);
  //    return false;
  //}
  nodehandle_.param("rate", max_pub_frequency_, 100.0);
  nodehandle_.param("frame", frame_id_, std::string("TrakstarBase"));
  nodehandle_.param("calibration_data", calibration_data_, std::string(""));

  // if calibration enabled then also publish raw data
  //_posearray_publisher = _node_handle.advertise<geometry_msgs::PoseArray>("poses_raw", 5);

  ROS_DEBUG_STREAM("Initialising TrakStar sensor system, this might take some time...");
  t_driver_->initialize(true);

  ROS_DEBUG_STREAM("Sensors Initialised");
  while (t_driver_->getInitStatus() == Trakstar::TRAKSTAR_STATUS_INITIALIZING)
  { /* WAIT FOR INITIALIZATION TO FINISH */
  }

  if (t_driver_->getInitStatus() == Trakstar::TRAKSTAR_STATUS_STARTED)
  {
    t_driver_->startPolling();
    ROS_DEBUG_STREAM("Sensors Initialised: Is polling " << t_driver_->isPolling());
  }
  else
  {
    CAROS_FATALERROR("Unable to initialise trakStar", TRAKSTAR_DRIVER_INITIALIZATION_FAIL);
    return false;
  }

  if (!PoseSensorServiceInterface::configureInterface())
  {
    CAROS_FATALERROR("The pose sensor service could not be configured correctly.", TRAKSTAR_POSESENSOR_CONFIGURE_FAIL);
    return false;
  }

  if (!ButtonSensorServiceInterface::configureInterface())
  {
    CAROS_FATALERROR("The buttonservice could not be configured correctly.", TRAKSTAR_BUTTONSERVICE_CONFIGURE_FAIL);
    return false;
  }

  setLoopRateFrequency(max_pub_frequency_);

  return true;
}

void TrakstarNode::runLoopHook()
{
  // tjek status of driver, also make sure it is initialized
  //if(t_driver_->inError() ) { }
  if (!t_driver_->isInitialized())
  {
    ROS_ERROR_STREAM("Driver is not initialized! restart node...");
  }

  std::vector<Trakstar::PoseData> pd = t_driver_->getData();
  geometry_msgs::PoseArray array;
  std_msgs::Bool buttonMsg;


  std::vector<Transform3D<> > transforms;
  std::vector<int > ids;
  std::vector<float > qualities;

  for (unsigned int i = 0; i < pd.size(); i++)
  {
    ids.push_back(i);
    transforms.push_back( Transform3D<>(pd[i].pos*0.001, pd[i].rot.toRotation3D()) );
    qualities.push_back(pd[i].quality);

    /*
    tsp.status = pd[i].status;
    tsp.valid = pd[i].valid;
    tsp.button = pd[i].analogButtonOn;
    if (pd[i].analogButtonOn)
      buttonMsg.data = 1;
    else
      buttonMsg.data = 0;

    msgs->poses.push_back(tsp);

    array.poses.push_back(tsp.pose);
    */
  }

  PoseSensorServiceInterface::publishPoses(transforms, ids, qualities );

}

