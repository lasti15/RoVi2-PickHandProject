#include <caros/gripper_si_proxy.h>

#include <caros/gripper_service_interface.h> /* provides GRIPPER_SERVICE_INTERFACE_SUB_NAMESPACE and the different msg \
                                                and srv types*/
#include <caros/common.h>
#include <caros/exceptions.h>

#include <caros_control_msgs/gripper_state.h>

/* TODO:
 * - Does it make sense to have a nodehandle or just use the non-object/instance versions of the function calls?
 *  ^--< With this simple implementation, where the subscribed callback will only be called when the user of this proxy
 *calls spin or otherwise invoke the callback handling, then it makes plenty of sense to enforce the user to provide a
 *nodehandle - otherwise this functionality will not work as intended.
 *  ^-- This also makes it the user's responsibility to ensure that the calls to getTimeStamp() and other interesting
 *getters are done on the same received GripperState i.e. not calling any spin or processing of the subscribing callback
 *queue... otherwise the data may be inconsistent.
 *  ^-- If using a thread to run the handleGripperState callback, then it would make sense to expose the reported
 *gripperState to the user or create a struct holding the same fields just with converted types such as rw::math::Q
 *instead of caros_common_msgs/q.
 *
 * - Would it make sense to default to persistent connections? and automatic reconnection?
 *
 * - See http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning for more information on turning this proxy into
 *a "selfhosted" ros node - so it spawns a thread that spins and handle the subscribed queue/callback - this can make it
 *possible to fully eliminate the knowledge of ROS from the user of this proxy (given that the ros::Time (header,
 *timestamp) type is eliminated by finding another type to represent the timestamps).
 * ^-- Also document these assumptions, so that users of this proxy easily understand how to properly use it.
 *
 * What to do when no gripper state data has been acquired yet?
 *   - Simply throw an exception stating that the data is invalid?
 *   - or how to properly signal that the data has not been updated at all?
 *   - what about timestamps?
 *
 * How to diagnose if the subscription is still valid and reinitiate a subscription? [ use subGripperState_() != 0 or
 *similar, but reinitiating requires creating a new object or is that supposed to never ever happen so the functionality
 *is missing? [ possibly interesting functions: getNumPublishers() and/or getTopic() ... ]
 * - Should there be a function to verify the subscription state/condition, and/or have every function that is being
 *invoked verify the state/condition of this object? Or should it just be the getters that verify that the subscription
 *is still active?
 *
 * The service calls can be extended with http://docs.ros.org/api/roscpp/html/classros_1_1ServiceClient.html :
 *waitForExistence(...) - taking an optional parameter specifying the timeout or just a boolean specifying whether or
 *not to block until the service becomes available...
 *
 * The user should have easy access to the error information regarding the state of the gripper / gripper caros node.
 *
 * Add more getters to allow the user to obtain more information from the reported gripper state. Maybe even allow the
 *user access to the whole gripper state with ROS types (the user would then have to properly handle the type
 *conversions). Also see comment above about creating a struct holding the converted gripperState fields.
 *
 * Provide synchronous versions of the service calls such as moveQ and gripQ. This would make a blocking call waiting
 *for the reported gripper state to show the proper values, but handleGripperState would never be run with the current
 *setup - requiring the subscription callback to be handled in its own thread, as described above (in one of the TODO
 *comments).
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
