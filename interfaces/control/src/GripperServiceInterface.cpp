#include <caros/GripperServiceInterface.hpp>
#include <caros/common.hpp>
#include <ros/ros.h>
#include <rw/common/Ptr.hpp>
#include <string>

/*
#define QUEUE_SIZE 5
*/

GripperServiceInterface::GripperServiceInterface(const std::string& serviceName):
  _nodeHnd( rw::common::ownedPtr(new ros::NodeHandle(serviceName) ) ),
	_gripperName(serviceName)

{
  initGripperService();
}


GripperServiceInterface::GripperServiceInterface(rw::common::Ptr<ros::NodeHandle> nodehandle):
	_nodeHnd( nodehandle ),
	_gripperName( nodehandle->getNamespace() )

{
  initGripperService();
}

/*
void GripperServiceInterface::publishState(const rw::math::Q& q, const rw::math::Q& dq) {
    GripperState state;

    state.header.stamp = ros::Time::now();

    state.q.data.resize( q.size() );
    state.dq.data.resize( dq.size() );

    for (size_t i = 0; i<q.size(); i++) {
    	state.q.data[i] = q(i);
    }
    for (size_t i = 0; i<dq.size(); i++) {
    	state.dq.data[i] = dq[i];
    }

    _gripperStatePublisher.publish( state );
}
*/

/** TODO
 * - Check for errors such as if _srvMoveQ is false (see e.g. http://docs.ros.org/api/roscpp/html/classros_1_1NodeHandle.html#ae659319707eb40e8ef302763f7d632da under returns)
 * - Properly handle the possible exception from advertise(Service) - do we want to provide e.g. exeception safety?
 */

void GripperServiceInterface::initGripperService() {
/*
  _gripperStatePublisher = _nodeHnd->advertise<GripperState>("GripperState", QUEUE_SIZE);
*/

  _srvMoveQ = _nodeHnd->advertiseService("moveQ", &GripperServiceInterface::moveQHandle, this);
  _srvGripQ = _nodeHnd->advertiseService("gripQ", &GripperServiceInterface::gripQHandle, this);
  _srvSetForceQ = _nodeHnd->advertiseService("setForceQ", &GripperServiceInterface::setForceQHandle, this);
  _srvSetVelocityQ = _nodeHnd->advertiseService("setVelocityQ", &GripperServiceInterface::setVelocityQHandle, this);
  _srvStopMovement = _nodeHnd->advertiseService("stopMovement", &GripperServiceInterface::stopMovementHandle, this);
}

bool GripperServiceInterface::moveQHandle(caros_control::GripperMoveQ::Request& request,
					  caros_control::GripperMoveQ::Response& response)
{
  return moveQ(caros::fromRos(request.q));
}

bool GripperServiceInterface::gripQHandle(caros_control::GripperGripQ::Request& request,
					  caros_control::GripperGripQ::Response& response)
{
  return gripQ(caros::fromRos(request.q));
}

bool GripperServiceInterface::setForceQHandle(caros_control::GripperSetForceQ::Request& request,
					      caros_control::GripperSetForceQ::Response& response)
{
  return setForceQ(caros::fromRos(request.force));
}

bool GripperServiceInterface::setVelocityQHandle(caros_control::GripperSetVelocityQ::Request& request,
						 caros_control::GripperSetVelocityQ::Response& response)
{
  return setVelocityQ(caros::fromRos(request.velocity));
}

bool GripperServiceInterface::stopMovementHandle(caros_control::GripperStopMovement::Request& request,
						 caros_control::GripperStopMovement::Response& response)
{
  return stopMovement();
}
