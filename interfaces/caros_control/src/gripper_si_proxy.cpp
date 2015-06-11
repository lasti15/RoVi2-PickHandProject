#include <caros/gripper_si_proxy.h>

/* Provides GRIPPER_SERVICE_INTERFACE_SUB_NAMESPACE and the different msg and srv types*/
#include <caros/gripper_service_interface.h>

#include <caros/common.h>
#include <caros/common_robwork.h>
#include <caros/exceptions.h>

#include <caros_control_msgs/gripper_state.h>

/* TODO:
 * Find a good way to handle the situation where a user tries to get some data e.g. getQ and no data has ever been
 *received yet - currently it will just return the default initialised values.
 *
 * Provide synchronous versions of the service calls such as moveQ and gripQ. This would make a blocking call waiting
 *for the reported gripper state to show the proper values, but handleGripperState would never be run with the current
 *setup (where the user has to invoke "ros spin") - requiring the subscription callback to be handled in its own thread.
 */

using namespace caros;

GripperSIProxy::GripperSIProxy(ros::NodeHandle nodehandle, const std::string& devname,
                               const bool usePersistentConnections)
    : nodehandle_(nodehandle),
      usePersistentConnections_(usePersistentConnections),
      rosNamespace_("/" + devname + "/" + GRIPPER_SERVICE_INTERFACE_SUB_NAMESPACE),
      srvMoveQ_(nodehandle_, "move_q", rosNamespace_, usePersistentConnections_),
      srvGripQ_(nodehandle_, "grip_q", rosNamespace_, usePersistentConnections_),
      srvSetForceQ_(nodehandle_, "set_force_q", rosNamespace_, usePersistentConnections_),
      srvSetVelocityQ_(nodehandle_, "set_velocity_q", rosNamespace_, usePersistentConnections_),
      srvStopMovement_(nodehandle_, "stop_movement", rosNamespace_, usePersistentConnections_)
{
  /* TODO:
   * Make the queue size into a parameter that can be configured - (hardcoded to 1 here)
   */
  subGripperState_ = nodehandle_.subscribe("gripper_state", 1, &GripperSIProxy::handleGripperState, this);
}

GripperSIProxy::~GripperSIProxy()
{
  /* Empty */
}

bool GripperSIProxy::moveQ(const rw::math::Q& q)
{
  caros_control_msgs::gripper_move_q srv;
  srv.request.q = caros::toRos(q);

  srvMoveQ_.call<caros_control_msgs::gripper_move_q>(srv);

  return srv.response.success;
}

bool GripperSIProxy::gripQ(const rw::math::Q& q)
{
  caros_control_msgs::gripper_grip_q srv;
  srv.request.q = caros::toRos(q);

  srvGripQ_.call<caros_control_msgs::gripper_grip_q>(srv);

  return srv.response.success;
}

bool GripperSIProxy::setForceQ(const rw::math::Q& q)
{
  caros_control_msgs::gripper_set_force_q srv;
  srv.request.force = caros::toRos(q);

  srvSetForceQ_.call<caros_control_msgs::gripper_set_force_q>(srv);

  return srv.response.success;
}

bool GripperSIProxy::setVelocityQ(const rw::math::Q& q)
{
  caros_control_msgs::gripper_set_velocity_q srv;
  srv.request.velocity = caros::toRos(q);

  srvSetVelocityQ_.call<caros_control_msgs::gripper_set_velocity_q>(srv);

  return srv.response.success;
}

bool GripperSIProxy::stopMovement()
{
  caros_control_msgs::gripper_stop_movement srv;

  srvStopMovement_.call<caros_control_msgs::gripper_stop_movement>(srv);

  return srv.response.success;
}

/* Hardcoded since the connections are not added to a collection that can easily be iterated */
void GripperSIProxy::closePersistentConnections()
{
  srvMoveQ_.shutdown();
  srvGripQ_.shutdown();
  srvSetForceQ_.shutdown();
  srvSetVelocityQ_.shutdown();
  srvStopMovement_.shutdown();
}

rw::math::Q GripperSIProxy::getQ()
{
  return caros::toRw(pSV_gripperState_.q);
}

rw::math::Q GripperSIProxy::getQd()
{
  return caros::toRw(pSV_gripperState_.dq);
}

rw::math::Q GripperSIProxy::getForce()
{
  return caros::toRw(pSV_gripperState_.force);
}

ros::Time GripperSIProxy::getTimeStamp()
{
  return pSV_gripperState_.header.stamp;
}

void GripperSIProxy::handleGripperState(const caros_control_msgs::gripper_state& state)
{
  pSV_gripperState_ = state;
}
