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


  node_handle_.param("ip", ip_, std::string("192.168.1.1"));
  node_handle_.param("port", port_, 49152);
  node_handle_.param("rate", publishRate_, 400);
  netft_ = ownedPtr(new NetFTLogging(ip_, port_));

  try {
    netft_->start();
  } catch (const std::exception& exp) {
    CAROS_FATALERROR("Unable to start communication with the NetFT sensor", NETFT_UNABLE_TO_START_COMMUNICATION);
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
    publish(rw::math::Wrench6D<>(data.data.first, data.data.second), "");
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


