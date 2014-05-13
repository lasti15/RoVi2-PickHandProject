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
 * \brief Convert the ROS message caros_common::Q to the RobWork type rw::math::Q
 * \param[in] q
 */
    rw::math::Q toRw(const caros_common::Q& q);

    caros_common::Q toRos(const rw::math::Q& q);

    geometry_msgs::Transform toRos(const rw::math::Transform3D<>& transform);

    rw::math::Transform3D<> toRw(const geometry_msgs::Transform& transform);

    rw::math::Transform3D<> toRw(const geometry_msgs::Pose& transform);

//! convert Wrench to Wrench6D
    rw::math::Wrench6D<> toRw(const geometry_msgs::Wrench& wrench);

//! convert Wrench6D to Wrench
    geometry_msgs::Wrench toRos(const rw::math::Wrench6D<>& w);

//! convert Twist to VelocitySrew6D
    rw::math::VelocityScrew6D<> toRw(const geometry_msgs::Twist& twist);

//! convert VelocitySrew6D to Twist
    geometry_msgs::Twist toRos(const rw::math::VelocityScrew6D<>& vs);
} // namespace

#endif /* CAROS_COMMON_H_ */
