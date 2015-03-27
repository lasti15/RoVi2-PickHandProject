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

GripperSIProxy::GripperSIProxy(ros::NodeHandle nodehandle, const std::string& devname) : nodeHnd_(nodehandle)
{
  std::ostringstream rosNamespace;
  rosNamespace << "/" << devname << "/" << GRIPPER_SERVICE_INTERFACE_SUB_NAMESPACE;

  srvMoveQ_ = nodeHnd_.serviceClient<caros_control_msgs::gripper_move_q>(rosNamespace.str() + "/move_q");
  srvGripQ_ = nodeHnd_.serviceClient<caros_control_msgs::gripper_grip_q>(rosNamespace.str() + "/grip_q");
  srvSetForceQ_ = nodeHnd_.serviceClient<caros_control_msgs::gripper_set_force_q>(rosNamespace.str() + "/set_force_q");
  srvSetVelocityQ_ =
      nodeHnd_.serviceClient<caros_control_msgs::gripper_set_velocity_q>(rosNamespace.str() + "/set_velocity_q");
  srvStopMovement_ =
      nodeHnd_.serviceClient<caros_control_msgs::gripper_stop_movement>(rosNamespace.str() + "/stop_movement");

  /* TODO:
   * Make the queue size into a parameter that can be configured - (hardcoded to 1 here)
   */
  subGripperState_ = nodeHnd_.subscribe("gripper_state", 1, &GripperSIProxy::handleGripperState, this);
}

GripperSIProxy::~GripperSIProxy()
{
  /* Empty */
}

bool GripperSIProxy::moveQ(const rw::math::Q& q)
{
  bool srvCallSuccess = false;
  caros_control_msgs::gripper_move_q srv;
  srv.request.q = caros::toRos(q);

  if (!srvMoveQ_.exists())
  {
    THROW_CAROS_UNAVAILABLE_SERVICE("The service " << srvMoveQ_.getService() << " does not exist.");
  }

  srvCallSuccess = srvMoveQ_.call(srv);
  if (!srvCallSuccess)
  {
    THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << srvMoveQ_.getService());
  }

  return srv.response.success;
}

bool GripperSIProxy::gripQ(const rw::math::Q& q)
{
  bool srvCallSuccess = false;
  caros_control_msgs::gripper_grip_q srv;
  srv.request.q = caros::toRos(q);

  if (!srvGripQ_.exists())
  {
    THROW_CAROS_UNAVAILABLE_SERVICE("The service " << srvGripQ_.getService() << " does not exist.");
  }

  srvCallSuccess = srvGripQ_.call(srv);
  if (!srvCallSuccess)
  {
    THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << srvGripQ_.getService());
  }

  return srv.response.success;
}

bool GripperSIProxy::setForceQ(const rw::math::Q& q)
{
  bool srvCallSuccess = false;
  caros_control_msgs::gripper_set_force_q srv;
  srv.request.force = caros::toRos(q);

  if (!srvSetForceQ_.exists())
  {
    THROW_CAROS_UNAVAILABLE_SERVICE("The service " << srvSetForceQ_.getService() << " does not exist.");
  }

  srvCallSuccess = srvSetForceQ_.call(srv);
  if (!srvCallSuccess)
  {
    THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << srvSetForceQ_.getService());
  }

  return srv.response.success;
}

bool GripperSIProxy::setVelocityQ(const rw::math::Q& q)
{
  bool srvCallSuccess = false;
  caros_control_msgs::gripper_set_velocity_q srv;
  srv.request.velocity = caros::toRos(q);

  if (!srvSetVelocityQ_.exists())
  {
    THROW_CAROS_UNAVAILABLE_SERVICE("The service " << srvSetVelocityQ_.getService() << " does not exist.");
  }

  srvCallSuccess = srvSetVelocityQ_.call(srv);
  if (!srvCallSuccess)
  {
    THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << srvSetVelocityQ_.getService());
  }

  return srv.response.success;
}

bool GripperSIProxy::stopMovement()
{
  bool srvCallSuccess = false;
  caros_control_msgs::gripper_stop_movement srv;

  if (!srvStopMovement_.exists())
  {
    THROW_CAROS_UNAVAILABLE_SERVICE("The service " << srvStopMovement_.getService() << " does not exist.");
  }

  srvCallSuccess = srvStopMovement_.call(srv);
  if (!srvCallSuccess)
  {
    THROW_CAROS_BAD_SERVICE_CALL("An error happened while calling the service " << srvStopMovement_.getService());
  }

  return srv.response.success;
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
