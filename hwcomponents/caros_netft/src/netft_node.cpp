#include <caros/netft_node.h>

#include <rw/rw.hpp>

USE_ROBWORK_NAMESPACE

using namespace caros;
using namespace robwork;
using namespace rwhw;


  //! constructor
NetFTNode::NetFTNode(const ros::NodeHandle& node_handle): 
	caros::CarosNodeServiceInterface(node_handle),
  caros::FTSensorServiceInterface(node_handle),
  node_handle_(node_handle) 
{
  
}

  //! destructor
NetFTNode::~NetFTNode() {
  netft_->stop();
}



bool NetFTNode::activateHook()
{
  ROS_DEBUG("Activate Hook");

  node_handle_.param("ip", ip_, std::string("192.168.100.2"));
  node_handle_.param("port", port_, 49152);
  node_handle_.param("rate", publishRate_, 50);
  setLoopRateFrequency(publishRate_);
  netft_ = ownedPtr(new NetFTLogging(ip_, port_));
  
  try {
    netft_->start();
  } catch (const std::exception& exp) {
    CAROS_FATALERROR("Unable to start communication with the NetFT sensor", NETFT_UNABLE_TO_START_COMMUNICATION);
    return false;
  }

  if (not FTSensorServiceInterface::configureInterface())
  {
    CAROS_FATALERROR("The CAROS FTSensorServiceInterface could not be configured correctly.",
                     NETFT_CAROS_GRIPPER_SERVICE_CONFIGURE_FAIL);
    return false;
  }

  return true;
}

bool NetFTNode::recoverHook(const std::string& error_msg, const int64_t error_code)
{
  /* TODO: */

  ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " has not been implemented yet!");
  ROS_BREAK();

  return false;
}

void NetFTNode::runLoopHook()
{
  try
  {
    if (netft_ == 0)
    {
      CAROS_FATALERROR("The NetFT device is not configured", NETFT_INTERNAL_ERROR);
      return;
    }

    
    NetFTLogging::NetFTData data = netft_->getAllData();       
    publish(rw::math::Wrench6D<>(data.data.first, data.data.second), "FT");
    std::cout<<"data = "<<data.data.first<<" "<<std::setprecision(16)<<data.timestamp<<std::endl;
    //publish(rw::math::Wrench6D<>(Vector3D<>(1,2,3), Vector3D<>(4,5,6)), "FT");
  }
  catch (const rw::common::Exception& exp)
  {
    CAROS_ERROR(exp.what(), NETFT_INTERNAL_ERROR);
    return;
  }
}

void NetFTNode::errorLoopHook()
{
  /* Stop the NetFTNode's current action(s) */
  if (netft_ == 0)
  {
    ROS_DEBUG_STREAM("The NetFT device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
  }
  else
  {
    netft_->stop();
  }
}

void NetFTNode::fatalErrorLoopHook()
{
  /* Stop the NetFTNode's current action(s) */
  if (netft_ == 0)
  {
    ROS_DEBUG_STREAM("The NetFT device was not configured when '" << __PRETTY_FUNCTION__ << "' was invoked!");
  }
  else
  {
    netft_->stop();
  }
}


