#ifndef CAROS_COMMON_H_
#define CAROS_COMMON_H_

#include <rw/math.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>

#include <caros_common_msgs/Q.h>
#include <caros_common_msgs/RWState.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

/**
 * \brief CAROS specific functionality
 */
namespace caros {
    /**
     * \addtogroup TypeConversion CAROS Type Conversion
     * Overloaded utility functions for converting between the different system types (e.g. from ROS to RobWork)
     * @{
     */

    /* TODO:
     * Better conversion descriptions? But it also is bad to hardcode the exact types in the documentation, so it has to be changed if e.g. the ROS/CAROS package is changed.
     */

    //! convert Q to Q
    rw::math::Q toRw(const caros_common_msgs::Q& q);

    //! convert Q to Q
    caros_common_msgs::Q toRos(const rw::math::Q& q);

    //! convert Transform3D to Transform
    geometry_msgs::Transform toRos(const rw::math::Transform3D<>& transform);

    //! convert Transform to Transform3D
    rw::math::Transform3D<> toRw(const geometry_msgs::Transform& transform);

    //! convert Pose to Transform3D
    rw::math::Transform3D<> toRw(const geometry_msgs::Pose& transform);

    //! convert Wrench to Wrench6D
    rw::math::Wrench6D<> toRw(const geometry_msgs::Wrench& wrench);

    //! convert Wrench6D to Wrench
    geometry_msgs::Wrench toRos(const rw::math::Wrench6D<>& w);

    //! convert Twist to VelocityScrew6D
    rw::math::VelocityScrew6D<> toRw(const geometry_msgs::Twist& twist);

    //! convert VelocityScrew6D to Twist
    geometry_msgs::Twist toRos(const rw::math::VelocityScrew6D<>& vs);

    //! convert float to float
    float toRos(const float value);

    //! convert bool to bool
    bool toRos(const bool value); 
 
    /* TODO:
     * Properly document these functions
     */
    void toRos(const rw::math::Transform3D<>& transform, geometry_msgs::Pose& pose);

    caros_common_msgs::RWState toRos(const rw::kinematics::State& state);

    void toRw(const caros_common_msgs::RWState& state, rw::kinematics::State& state_dst);

    rw::kinematics::State toRw(const caros_common_msgs::RWState& state, rw::models::WorkCell::Ptr wc);


    /**
     * @} end of group
     */

    /**
     * @brief gets the workcell from parameter server.
     *
     * The workcell should be placed in /caros/workcell on the
     * parameter server.
     *
     * @note requires that ROS is initialized
     * @return the WorkCell or NULL
     */
    rw::models::WorkCell::Ptr getWorkCell();

    /**
     * @brief gets the workcell from parameter server.
     *
     * @note requires that ROS is initialized
     * @param paramname [in] the name of the variable on the parameter server
     * @return the WorkCell or NULL
     */
    rw::models::WorkCell::Ptr getWorkCell(const std::string& paramname);

    /**
     * @brief get current stateinformation of the workcell
     * @return
     */
    rw::common::Ptr< rw::kinematics::State > getState( );

} // namespace

#endif /* CAROS_COMMON_H_ */
