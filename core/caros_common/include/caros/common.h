#ifndef CAROS_COMMON_COMMON_H
#define CAROS_COMMON_COMMON_H

#include <rw/math.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>

#include <caros_common_msgs/q.h>
#include <caros_common_msgs/rw_state.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <stdexcept>

/**
 * \brief CAROS specific functionality
 */
namespace caros
{
/**
 * \addtogroup TypeConversion CAROS Type Conversion
 * Overloaded utility functions for converting between the different system types (e.g. from ROS to RobWork)
 * If a type can be converted to more types, then the most obvious/direct type conversion is having the toRos or toRw
 * signature, while the alternative types will be toRosType and toRwType.
 * @{
 */

/* TODO:
 * Better conversion descriptions? But it also is bad to hardcode the exact types in the documentation, so it has to be
 * changed if e.g. the ROS/CAROS package is changed.
 */

//! convert Q to Q
rw::math::Q toRw(const caros_common_msgs::q& q);

//! convert Q to Q
caros_common_msgs::q toRos(const rw::math::Q& q);

//! convert Transform3D to Transform
geometry_msgs::Transform toRos(const rw::math::Transform3D<>& transform);

//! convert Transform to Transform3D
rw::math::Transform3D<> toRw(const geometry_msgs::Transform& transform);

/**
 * @brief convert Transform3D to Pose
 *
 * @note This function has the type appended i.e. toRosPose.
 * This is because the ROS pose is also represented as a Transform3D in RobWork.
 */
geometry_msgs::Pose toRosPose(const rw::math::Transform3D<>& transform);

//! convert Pose to Transform3D
rw::math::Transform3D<> toRw(const geometry_msgs::Pose& pose);

//! convert Wrench to Wrench6D
rw::math::Wrench6D<> toRw(const geometry_msgs::Wrench& wrench);

//! convert Wrench6D to Wrench
geometry_msgs::Wrench toRos(const rw::math::Wrench6D<>& w);

//! convert Twist to VelocityScrew6D
rw::math::VelocityScrew6D<> toRw(const geometry_msgs::Twist& twist);

//! convert VelocityScrew6D to Twist
geometry_msgs::Twist toRos(const rw::math::VelocityScrew6D<>& vs);

//! convert double to double
double toRos(const double value);

//! convert double to double
double toRw(const double value);

//! convert double to ros float
float toRosFloat(const double value);

//! convert float to ros double
double toRosDouble(const float value);

//! convert float to float
float toRos(const float value);

//! convert float to float
float toRw(const float value);

//! convert bool to bool
bool toRos(const bool value);

//! convert bool to bool
bool toRw(const bool value);

/* TODO:
 * Properly document these functions
 */

/**
 * @brief convert RobWork state to rw_state
 *
 * @note The full state can't be serialised yet. Endianness is not being handled properly.
 */
caros_common_msgs::rw_state toRos(const rw::kinematics::State& state);

/**
 * @brief Convert rw_state to RobWork state
 *
 * @note Requires a non-empty state (state) to copy the state information into (not everything can be serialized yet, so
 *this is basically just a way to transfer state changes across ROS nodes). Endianness is not being handled properly.
 */
void toRw(const caros_common_msgs::rw_state& stateRos, rw::kinematics::State& state);

/**
 * @brief Convert rw_state to RobWork state
 *
 * @note Endianness is not being handled properly. Uses the default state from the provided workcell ptr and populates
 *it accordingly to the content of rw_state.
 */
rw::kinematics::State toRw(const caros_common_msgs::rw_state& state, const rw::models::WorkCell::Ptr wc);

/**
 * @}
 */

/**
 * \addtogroup Utility CAROS Utility Functions
 * Utility functions for various CAROS related tasks, such as obtaining a workcell
 * @{
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
 * @note requires that ROS is initialized. Currently implemented as a singleton, so once a workcell is found and loaded
 *properly, then it will keep returning that workcell.
 * @param paramname [in] the name of the variable on the parameter server
 * @return the WorkCell or NULL
 */
rw::models::WorkCell::Ptr getWorkCell(const std::string& paramname);

/**
 * @brief get current state information of the workcell
 * @return pointer to new state or NULL if no state information is available.
 */
rw::common::Ptr<rw::kinematics::State> getState();

/**
 * @brief Verify that the value is within the range [min;max]. If the value is not within the range, then appropriate
exceptions will be thrown - behaving mostly as a runtime assertion.
 * @param[in] value the value to be compared
 * @param[in] min the lower boundary of the range
 * @param[in] max the upper boundary of the range
 *
 * @throws std::invalid_argument if any of the supplied values are NaN's
 * @throws std::range_error if the value is not within the specified range [min;max]
 */
/* Not doing this as a do {...} while(0) macro, as there then wouldn't be any type enforcement */
void verifyValueIsWithin(const float& value, const float& min, const float& max);

/**
 * @}
 */
}  // namespace

#endif /* CAROS_COMMON_COMMON_H */
