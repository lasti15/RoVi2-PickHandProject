#include <caros/common.h>

#include <rw/math.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <caros_common_msgs/Q.h>
#include <caros_common_msgs/get_rw_state.h>
#include <caros_common_msgs/rw_state.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <ros/ros.h>

#include <cstdint>

namespace caros
{
/************************************************************************
 * Notes:
 * These conversion functions can be templated to work with other scalar
 * types than the default 'double'.
 * But sticking with the default to keep things simple.
 ************************************************************************/

/************************************************************************
 * State
 ************************************************************************/
caros_common_msgs::rw_state toRos(const rw::kinematics::State& state)
{
  ROS_DEBUG_STREAM("RobWork state to ROS rw_state -begin-");
  caros_common_msgs::rw_state res;
  /* Preallocate the size */
  auto stateDataSize = state.getStateStructure()->getStateData().size();
  ROS_DEBUG_STREAM("stateDataSize: " << stateDataSize);
  res.state_data.reserve(stateDataSize);

  for (const auto& data : state.getStateStructure()->getStateData())
  {
    caros_common_msgs::rw_state_data resData;
    resData.name = data->getName();
    resData.size = data->size() * sizeof(double); /* This hardcoded sizeof should be provided by the 'data->' object, as the type used for storing the state data is highly implementation specific */
    ROS_DEBUG_STREAM("resData name: " << resData.name);
    ROS_DEBUG_STREAM("resData size: " << resData.size << " where data->size = " << data->size() << " and sizeof(double) = " << sizeof(double));
    /* Preallocate the size */
    resData.data.reserve(resData.size);

    for (std::size_t index = 0; index < resData.size; ++index)
    {
      /* Converting to uint8_t because the data in the state should just be considered a memory space and not necessarily consist of doubles, as it could as well be booleans and other sort of types */
      std::uint8_t binaryData = reinterpret_cast<const std::uint8_t*>(data->getData(state))[index];
      ROS_DEBUG_STREAM("binaryData[" << index << "] = " << std::hex << std::setw(2) << static_cast<std::uint32_t>(binaryData));
      resData.data.push_back(binaryData);
    }
    res.state_data.push_back(resData);
  }

  ROS_DEBUG_STREAM("RobWork state to ROS rw_state -end-");
  return res;
}

void toRw(const caros_common_msgs::rw_state& stateRos, rw::kinematics::State& state)
{
  ROS_DEBUG_STREAM("ROS rw_state to RobWork state -begin-");
  for (const auto& stateRosData : stateRos.state_data)
  {
    auto data = state.getStateStructure()->findData(stateRosData.name);
    if ((data->size() * sizeof(double)) == stateRosData.data.size())
    {
      ROS_DEBUG_STREAM("data name: " << stateRosData.name);
      ROS_DEBUG_STREAM("data size: " << stateRosData.data.size());
      // Place the stateRosData.data in a continuous memory space
      std::uint8_t rawData[stateRosData.data.size()];
      for (std::size_t index = 0; index < stateRosData.data.size(); ++index)
      {
        rawData[index] = stateRosData.data.at(index);
        ROS_DEBUG_STREAM("rawData[" << index << "] = " << std::hex << std::setw(2) << static_cast<std::uint32_t>(rawData[index]));
      }
      data->setData(state, reinterpret_cast<const double *>(rawData));
    }
    else
    {
      ROS_ERROR_STREAM("Mismatch in data (i.e. payload) lengths between current state and the serialised state from ROS: ros_state_name=" << stateRosData.name << " ros_state_size=" << stateRosData.data.size() * sizeof(double) << " current_state_size=" << data->size());
    }
  }
  ROS_DEBUG_STREAM("ROS rw_state to RobWork state -end-");
}

rw::kinematics::State toRw(const caros_common_msgs::rw_state& state_ros, const rw::models::WorkCell::Ptr wc)
{
  rw::kinematics::State state = wc->getDefaultState();

  toRw(state_ros, state);

  return state;
}

/************************************************************************
 * Q
 ************************************************************************/
rw::math::Q toRw(const caros_common_msgs::Q& q)
{
  rw::math::Q res(q.data.size());
  for (std::size_t i = 0; i < q.data.size(); ++i)
  {
    res(i) = q.data[i];
  }
  return res;
}

caros_common_msgs::Q toRos(const rw::math::Q& q)
{
  caros_common_msgs::Q res;
  res.data.resize(q.size());
  for (std::size_t i = 0; i < q.size(); ++i)
  {
    res.data[i] = static_cast<double>(q(i));
  }
  return res;
}

/************************************************************************
 * Transform
 ************************************************************************/
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

rw::math::Transform3D<> toRw(const geometry_msgs::Transform& transform)
{
  rw::math::Vector3D<> p(transform.translation.x, transform.translation.y, transform.translation.z);
  rw::math::Quaternion<> q(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
  return rw::math::Transform3D<>(p, q);
}

/************************************************************************
 * Pose
 *
 * Because the RobWork representation of a ROS pose is also a transform
 * then the function to convert to a ROS pose has the type appended i.e.
 * toRosPose
 * This may change in the future if a better way to streamline the
 * conversion functions is found with acceptable tradeoffs.
 ************************************************************************/
geometry_msgs::Pose toRosPose(const rw::math::Transform3D<>& transform)
{
  rw::math::Quaternion<> q(transform.R());
  geometry_msgs::Pose result;
  result.position.x = transform.P()(0);
  result.position.y = transform.P()(1);
  result.position.z = transform.P()(2);

  result.orientation.x = q(0);
  result.orientation.y = q(1);
  result.orientation.z = q(2);
  result.orientation.w = q(3);

  return result;
}

rw::math::Transform3D<> toRw(const geometry_msgs::Pose& pose)
{
  rw::math::Vector3D<> p(pose.position.x, pose.position.y, pose.position.z);
  rw::math::Quaternion<> q(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                           pose.orientation.w);
  return rw::math::Transform3D<>(p, q);
}

/************************************************************************
 * Wrench
 ************************************************************************/
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

rw::math::Wrench6D<> toRw(const geometry_msgs::Wrench& wrench)
{
  rw::math::Wrench6D<> w(wrench.force.x, wrench.force.y, wrench.force.z, wrench.torque.x, wrench.torque.y,
                         wrench.torque.z);
  return w;
}

/************************************************************************
 * Twist / VelocityScrew
 ************************************************************************/
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

/************************************************************************
 * Double
 ************************************************************************/
double toRos(const double value)
{
  return value;
}

double toRw(const double value)
{
  return value;
}

/************************************************************************
 * Float
 ************************************************************************/
float toRos(const float value)
{
  return value;
}

float toRw(const float value)
{
  return value;
}

/************************************************************************
 * Boolean
 ************************************************************************/
bool toRos(const bool value)
{
  return value;
}

bool toRw(const bool value)
{
  return value;
}

/************************************************************************
 *
 ************************************************************************/
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
  if (!ros::service::exists("/caros/get_workcell_state", true))
  {
    ROS_ERROR_STREAM("There are no registered state sources!");
    return NULL;
  }

  rw::models::WorkCell::Ptr wc = getWorkCell();

  ros::ServiceClient global = node.serviceClient<caros_common_msgs::get_rw_state>("/caros/get_workcell_state");

  caros_common_msgs::get_rw_state service;

  global.call(service);

  rw::kinematics::State state = caros::toRw(service.response.state, wc);

  return ownedPtr(new rw::kinematics::State(state));
}

}  // namespace
