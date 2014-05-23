#ifndef CAROS_COMMON_H_
#define CAROS_COMMON_H_

#include <rw/math.hpp>
#include <caros_common/Q.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>


/**
 * \brief This is documentation for the namespace caros
 */
namespace caros {
/**
 * \brief Convert the ROS message caros::Q to the RobWork type rw::math::Q
 * \param[in] q
 */
inline rw::math::Q toRw(const caros_common::Q& q)
{
    rw::math::Q res(q.data.size());
    for (size_t i = 0; i < q.data.size(); ++i) {
        res(i) = q.data[i];
    }
    return res;
}

inline caros_common::Q toRos(const rw::math::Q& q)
{
    caros_common::Q res;
    res.data.resize(q.size());
    for (size_t i = 0; i < q.size(); ++i) {
        res.data[i] = static_cast<double>(q(i));
    }
    return res;
}

inline geometry_msgs::Transform toRos(const rw::math::Transform3D<>& transform)
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

inline rw::math::Transform3D<> toRw(const geometry_msgs::Transform& transform) {
    rw::math::Vector3D<> p(transform.translation.x, transform.translation.y, transform.translation.z);
    rw::math::Quaternion<> q(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
    return rw::math::Transform3D<>(p, q);
}

inline rw::math::Transform3D<> toRw(const geometry_msgs::Pose& transform) {
    rw::math::Vector3D<> p(transform.position.x, transform.position.y, transform.position.z);
    rw::math::Quaternion<> q(transform.orientation.x, transform.orientation.y, transform.orientation.z, transform.orientation.w);
    return rw::math::Transform3D<>(p, q);
}

//! convert Wrench to Wrench6D
inline rw::math::Wrench6D<> toRw(const geometry_msgs::Wrench& wrench) {
    rw::math::Wrench6D<> w(wrench.force.x, wrench.force.y, wrench.force.z, wrench.torque.x, wrench.torque.y, wrench.torque.z);
    return w;
}

//! convert Wrench6D to Wrench
inline geometry_msgs::Wrench toRos(const rw::math::Wrench6D<>& w) {
    geometry_msgs::Wrench wrench;
    wrench.force.x = w.force()[0];
    wrench.force.y = w.force()[1];
    wrench.force.z = w.force()[2];
    wrench.torque.x = w.torque()[0];
    wrench.torque.y = w.torque()[1];
    wrench.torque.z = w.torque()[2];
    return wrench;
}

//! convert Twist to VelocitySrew6D
inline rw::math::VelocityScrew6D<> toRw(const geometry_msgs::Twist& twist) {
    rw::math::VelocityScrew6D<> vs(twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z);
    return vs;
}
//! convert VelocitySrew6D to Twist
inline geometry_msgs::Twist toRos(const rw::math::VelocityScrew6D<>& vs) {
    geometry_msgs::Twist twist;
    twist.linear.x = vs(0);
    twist.linear.y = vs(1);
    twist.linear.z = vs(2);
    twist.angular.x = vs(3);
    twist.angular.y = vs(4);
    twist.angular.z = vs(5);
    return twist;
}


}

#endif /* CAROS_COMMON_H_ */
