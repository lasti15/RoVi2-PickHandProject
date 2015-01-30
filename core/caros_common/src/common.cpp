#include <caros/common.h>

#include <rw/math.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <caros_common_msgs/Q.h>
#include <caros_common_msgs/GetRWState.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <ros/ros.h>

namespace caros
{

caros_common_msgs::RWState toRos(const rw::kinematics::State& state)
{
  caros_common_msgs::RWState res;
  BOOST_FOREACH(boost::shared_ptr<rw::kinematics::StateData> data, state.getStateStructure()->getStateData())
  {
    std::string name = data->getName();
    int size = data->size() * sizeof(double);

    res.statedata_names.push_back(name);
    res.statedata_sizes.push_back(size);
    for (int i = 0; i < size; i++)
    {
      res.statedata_q.push_back(((const boost::uint8_t*)data->getData(state))[i]);
    }
  }

  return res;
}

void toRw(const caros_common_msgs::RWState& state_ros, rw::kinematics::State& state)
{
  int idx = 0;
  for (size_t i = 0; i < state_ros.statedata_names.size(); i++)
  {

    std::string name = state_ros.statedata_names[i];
    int size = state_ros.statedata_sizes[i];
    boost::shared_ptr<rw::kinematics::StateData> data = state.getStateStructure()->findData(name);
    // todo: verify that data
    if (data->size() * sizeof(double) == size)
    {
      for (int i = 0; i < size; i++)
      {
        data->setData(state, (double*)(&state_ros.statedata_q[idx]));
      }
    }
    else
    {
      ROS_ERROR_STREAM("Mismatch in statedata lengths between current state and serialized state! For "
                       << name << " " << data->size() * 4 << "==" << size);
    }

    idx += size;
  }
}

rw::kinematics::State toRw(const caros_common_msgs::RWState& state_ros, rw::models::WorkCell::Ptr wc)
{
  rw::kinematics::State state = wc->getDefaultState();

  toRw(state_ros, state);

  return state;
}

rw::math::Q toRw(const caros_common_msgs::Q& q)
{
  rw::math::Q res(q.data.size());
  for (size_t i = 0; i < q.data.size(); ++i)
  {
    res(i) = q.data[i];
  }
  return res;
}

caros_common_msgs::Q toRos(const rw::math::Q& q)
{
  caros_common_msgs::Q res;
  res.data.resize(q.size());
  for (size_t i = 0; i < q.size(); ++i)
  {
    res.data[i] = static_cast<double>(q(i));
  }
  return res;
}

geometry_msgs::Transform toRos(const rw::math::Transform3D<>& transform)
{
  rw::math::Quaternion<> q(transform.R());
  geometry_msgs::Transform result;
  result.translation.x = transform.P()(0);
  result.translation.y = transform.P()(1);
  result.translation.z = transform.P()(2);

  result.rotation.x = q(0);
  result.rotation.y = q(1);
  result.rotation.z = q(2);
  result.rotation.w = q(3);

  return result;
}

void toRos(const rw::math::Transform3D<>& transform, geometry_msgs::Pose& pose)
{
  rw::math::Quaternion<> q(transform.R());

  pose.position.x = transform.P()(0);
  pose.position.y = transform.P()(1);
  pose.position.z = transform.P()(2);

  pose.orientation.x = q(0);
  pose.orientation.y = q(1);
  pose.orientation.z = q(2);
  pose.orientation.w = q(3);
}

rw::math::Transform3D<> toRw(const geometry_msgs::Transform& transform)
{
  rw::math::Vector3D<> p(transform.translation.x, transform.translation.y, transform.translation.z);
  rw::math::Quaternion<> q(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
  return rw::math::Transform3D<>(p, q);
}

rw::math::Transform3D<> toRw(const geometry_msgs::Pose& transform)
{
  rw::math::Vector3D<> p(transform.position.x, transform.position.y, transform.position.z);
  rw::math::Quaternion<> q(transform.orientation.x, transform.orientation.y, transform.orientation.z,
                           transform.orientation.w);
  return rw::math::Transform3D<>(p, q);
}

rw::math::Wrench6D<> toRw(const geometry_msgs::Wrench& wrench)
{
  rw::math::Wrench6D<> w(wrench.force.x, wrench.force.y, wrench.force.z, wrench.torque.x, wrench.torque.y,
                         wrench.torque.z);
  return w;
}

geometry_msgs::Wrench toRos(const rw::math::Wrench6D<>& w)
{
  geometry_msgs::Wrench wrench;
  wrench.force.x = w.force()[0];
  wrench.force.y = w.force()[1];
  wrench.force.z = w.force()[2];
  wrench.torque.x = w.torque()[0];
  wrench.torque.y = w.torque()[1];
  wrench.torque.z = w.torque()[2];
  return wrench;
}

rw::math::VelocityScrew6D<> toRw(const geometry_msgs::Twist& twist)
{
  rw::math::VelocityScrew6D<> vs(twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y,
                                 twist.angular.z);
  return vs;
}

geometry_msgs::Twist toRos(const rw::math::VelocityScrew6D<>& vs)
{
  geometry_msgs::Twist twist;
  twist.linear.x = vs(0);
  twist.linear.y = vs(1);
  twist.linear.z = vs(2);
  twist.angular.x = vs(3);
  twist.angular.y = vs(4);
  twist.angular.z = vs(5);
  return twist;
}

float toRos(const float value)
{
  return value;
}

bool toRos(const bool value)
{
  return value;
}

rw::models::WorkCell::Ptr getWorkCell()
{
  return getWorkCell("/caros/workcell");
}

rw::models::WorkCell::Ptr getWorkCell(const std::string& paramname)
{
  static rw::models::WorkCell::Ptr wc;
  if (wc == NULL)
  {
    ros::NodeHandle node("~");
    std::string workcellFile;
    bool paramFound;
    paramFound = node.getParam(paramname, workcellFile);
    if (!paramFound)
    {
      ROS_ERROR_STREAM("No such parameter on the parameter server: " << paramname);
      return NULL;
    }
    else if (workcellFile.empty())
    {
      ROS_ERROR_STREAM("The value of the parameter is empty!");
      return NULL;
    }
    ROS_DEBUG_STREAM("loading file: " << workcellFile);
    try
    {
      wc = rw::loaders::WorkCellFactory::load(workcellFile);
    }
    catch (const std::exception& exp)
    {
      ROS_ERROR_STREAM("Unable to open workcell file ");
      ROS_ERROR_STREAM("Error: " << exp.what());
      return NULL;
    }
  }
  return wc;
}

rw::common::Ptr<rw::kinematics::State> getState()
{
  // currently we always get the state from a service and not a topic
  ros::NodeHandle node("~");
  if (!ros::service::exists("/caros/getworkcellstate", true))
  {
    ROS_ERROR_STREAM("There are no registered state sources! Using default state.");
    return NULL;
  }

  rw::models::WorkCell::Ptr wc = getWorkCell();

  ros::ServiceClient global = node.serviceClient<caros_common_msgs::GetRWState>("/caros/getworkcellstate");

  caros_common_msgs::GetRWState service;

  global.call(service);

  rw::kinematics::State state = caros::toRw(service.response.state, wc);

  return ownedPtr(new rw::kinematics::State(state));
}

}  // namespace
