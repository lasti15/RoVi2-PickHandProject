#include <caros/GripperServiceInterface.hpp>
#include <caros/common.h>

#include <caros_control_msgs/gripper_state.h>

#include <rw/math/Q.hpp>

#include <ros/ros.h>

#include <string>

using namespace caros;

GripperServiceInterface::GripperServiceInterface(const ros::NodeHandle& nodehandle)
    : nodeHandle_(nodehandle, GRIPPER_SERVICE_INTERFACE_SUB_NAMESPACE)
{
  /* Do nothing */
}

GripperServiceInterface::GripperServiceInterface()
{
  /* Do nothing */
  ROS_FATAL_STREAM(
      "The empty constructor of the GripperServiceInterface should never be called! This is undefined behaviour.");
}

GripperServiceInterface::~GripperServiceInterface()
{
  /* Currently no special things to clean up */
}

bool GripperServiceInterface::configureInterface()
{
  return initService();
}

bool GripperServiceInterface::initService()
{
  if (gripperStatePublisher_ || srvMoveQ_ || srvGripQ_ || srvSetForceQ_ || srvSetVelocityQ_ || srvStopMovement_)
  {
    ROS_WARN_STREAM(
        "Reinitialising one or more GripperServiceInterface services or publishers. If this is not fully intended then "
        "this should be considered a bug!");
  }

  gripperStatePublisher_ =
      nodeHandle_.advertise<caros_control_msgs::gripper_state>("gripper_state", GRIPPER_STATE_PUBLISHER_QUEUE_SIZE);
  ROS_ERROR_STREAM_COND(!gripperStatePublisher_, "The GripperState publisher is empty!");

  srvMoveQ_ = nodeHandle_.advertiseService("move_q", &GripperServiceInterface::moveQHandle, this);
  ROS_ERROR_STREAM_COND(!srvMoveQ_, "The move_q service is empty!");

  srvGripQ_ = nodeHandle_.advertiseService("grip_q", &GripperServiceInterface::gripQHandle, this);
  ROS_ERROR_STREAM_COND(!srvGripQ_, "The grip_q service is empty!");

  srvSetForceQ_ = nodeHandle_.advertiseService("set_force_q", &GripperServiceInterface::setForceQHandle, this);
  ROS_ERROR_STREAM_COND(!srvSetForceQ_, "The set_force_q service is empty!");

  srvSetVelocityQ_ = nodeHandle_.advertiseService("set_velocity_q", &GripperServiceInterface::setVelocityQHandle, this);
  ROS_ERROR_STREAM_COND(!srvSetVelocityQ_, "The set_velocity_q service is empty!");

  srvStopMovement_ = nodeHandle_.advertiseService("stop_movement", &GripperServiceInterface::stopMovementHandle, this);
  ROS_ERROR_STREAM_COND(!srvStopMovement_, "The stop_movement service is empty!");

  /* Verify that the various ROS services have actually been created properly */
  if (gripperStatePublisher_ && srvMoveQ_ && srvGripQ_ && srvSetForceQ_ && srvSetVelocityQ_ && srvStopMovement_)
  {
    /* Everything seems to be properly initialised */
    ROS_DEBUG_STREAM("All GripperServiceInterface publishers and services appear to have been properly initialised");
  }
  else
  {
    ROS_ERROR_STREAM(
        "The gripper service could not be properly initialised - one or more ros services or publishers failed to be "
        "properly initialised.");
    return false;
  }
  return true;
}

void GripperServiceInterface::publishState(const rw::math::Q& q, const rw::math::Q& dq, const rw::math::Q& jointforce,
                                           bool isMoving, bool isBlocked, bool isStopped, bool isEstopped)
{
  caros_control_msgs::gripper_state state;

  state.header.stamp = ros::Time::now();

  state.q = caros::toRos(q);
  state.dq = caros::toRos(dq);
  state.force = caros::toRos(jointforce);
  state.isMoving = isMoving;
  state.isBlocked = isBlocked;
  state.isStopped = isStopped;
  state.estopped = isEstopped;

  if (gripperStatePublisher_)
  {
    gripperStatePublisher_.publish(state);
  }
  else
  {
    ROS_ERROR_STREAM(
        "The gripperStatePublisher_ is empty - trying to publish gripper state with a non-working "
        "GripperSreviceInterface object.");
  }
}

bool GripperServiceInterface::moveQHandle(caros_control_msgs::gripper_move_q::Request& request,
                                          caros_control_msgs::gripper_move_q::Response& response)
{
  response.success = moveQ(caros::toRw(request.q));
  return true;
}

bool GripperServiceInterface::gripQHandle(caros_control_msgs::gripper_grip_q::Request& request,
                                          caros_control_msgs::gripper_grip_q::Response& response)
{
  response.success = gripQ(caros::toRw(request.q));
  return true;
}

bool GripperServiceInterface::setForceQHandle(caros_control_msgs::gripper_set_force_q::Request& request,
                                              caros_control_msgs::gripper_set_force_q::Response& response)
{
  response.success = setForceQ(caros::toRw(request.force));
  return true;
}

bool GripperServiceInterface::setVelocityQHandle(caros_control_msgs::gripper_set_velocity_q::Request& request,
                                                 caros_control_msgs::gripper_set_velocity_q::Response& response)
{
  response.success = setVelocityQ(caros::toRw(request.velocity));
  return true;
}

bool GripperServiceInterface::stopMovementHandle(caros_control_msgs::gripper_stop_movement::Request& request,
                                                 caros_control_msgs::gripper_stop_movement::Response& response)
{
  response.success = stopMovement();
  return true;
}
