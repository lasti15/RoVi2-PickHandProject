
#include <caros/GripperServiceInterface.hpp>
#include <caros/common.hpp>

#include <caros_control/GripperState.h>

#include <ros/ros.h>
#include <rw/common/Ptr.hpp>
#include <string>

// size for publishing GripperState
#define QUEUE_SIZE 5

namespace caros {

  GripperServiceInterface::GripperServiceInterface(const std::string& serviceName):
    _nodeHnd( rw::common::ownedPtr(new ros::NodeHandle(serviceName) ) ),
    _gripperName(serviceName),
    _initialized(false)

  {
    //initGripperService();
  }


  GripperServiceInterface::GripperServiceInterface(rw::common::Ptr<ros::NodeHandle> nodehandle):
    _nodeHnd( nodehandle ),
    _gripperName( nodehandle->getNamespace() ),
    _initialized(false)
  {
    //initGripperService();
  }

  GripperServiceInterface::GripperServiceInterface():
        _initialized(false)
  {

  }

  GripperServiceInterface::~GripperServiceInterface(){}

void GripperServiceInterface::publishState(
        const rw::math::Q& q,
        const rw::math::Q& dq,
        const rw::math::Q& jointforce,
        bool isMoving, bool isBlocked, bool isStopped, bool isEstopped)
{
    caros_control::GripperState state;

    state.header.stamp = ros::Time::now();

    state.q = caros::toRos(q);
    state.dq = caros::toRos(dq);
    state.force = caros::toRos(jointforce);
    state.isMoving = isMoving;
    state.isBlocked = isBlocked;
    state.isStopped = isStopped;
    state.estopped = isEstopped;

    _gripperStatePublisher.publish(state);
}


 bool GripperServiceInterface::configureGripperService(){
     if(_initialized)
         return false;
     initGripperService();
     _initialized = true;
     return true;
 }

 bool GripperServiceInterface::cleanupGripperService(){
     if(!_initialized)
         return true;
     _srvMoveQ.shutdown();
     _srvGripQ.shutdown();
     _srvSetForceQ.shutdown();
     _srvSetVelocityQ.shutdown();
     _srvStopMovement.shutdown();
     _initialized = false;
     return true;
 }


  /** TODO
   * - Check for errors such as if _srvMoveQ is false (see e.g. http://docs.ros.org/api/roscpp/html/classros_1_1NodeHandle.html#ae659319707eb40e8ef302763f7d632da under returns)
   * - Properly handle the possible exception from advertise(Service) - do we want to provide e.g. exeception safety?
   */

  void GripperServiceInterface::initGripperService() {
    _gripperStatePublisher = _nodeHnd->advertise<caros_control::GripperState>("GripperState", QUEUE_SIZE);


    _srvMoveQ = _nodeHnd->advertiseService("move_q", &GripperServiceInterface::moveQHandle, this);
    _srvGripQ = _nodeHnd->advertiseService("grip_q", &GripperServiceInterface::gripQHandle, this);
    _srvSetForceQ = _nodeHnd->advertiseService("set_force_q", &GripperServiceInterface::setForceQHandle, this);
    _srvSetVelocityQ = _nodeHnd->advertiseService("set_velocity_q", &GripperServiceInterface::setVelocityQHandle, this);
    _srvStopMovement = _nodeHnd->advertiseService("stop_movement", &GripperServiceInterface::stopMovementHandle, this);
    _initialized = true;
  }

  bool GripperServiceInterface::moveQHandle(caros_control::GripperMoveQ::Request& request,
					    caros_control::GripperMoveQ::Response& response)
  {
      response.success = moveQ(caros::fromRos(request.q));
      return true;
  }

  bool GripperServiceInterface::gripQHandle(caros_control::GripperGripQ::Request& request,
					    caros_control::GripperGripQ::Response& response)
  {
      response.success = gripQ(caros::fromRos(request.q));
      return true;
  }

  bool GripperServiceInterface::setForceQHandle(caros_control::GripperSetForceQ::Request& request,
						caros_control::GripperSetForceQ::Response& response)
  {
      response.success = setForceQ(caros::fromRos(request.force));
    return true;
  }

  bool GripperServiceInterface::setVelocityQHandle(caros_control::GripperSetVelocityQ::Request& request,
						   caros_control::GripperSetVelocityQ::Response& response)
  {
      response.success = setVelocityQ(caros::fromRos(request.velocity));
      return true;
  }

  bool GripperServiceInterface::stopMovementHandle(caros_control::GripperStopMovement::Request& request,
						   caros_control::GripperStopMovement::Response& response)
  {
      response.success = stopMovement();
      return true;
  }

} // namespace
