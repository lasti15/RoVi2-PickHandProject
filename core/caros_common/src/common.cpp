#include <caros/common.hpp>

#include <rw/math.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <caros_common_msgs/Q.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <ros/ros.h>

namespace caros {

    rw::math::Q toRw(const caros_common_msgs::Q& q)
    {
        rw::math::Q res(q.data.size());
        for (size_t i = 0; i < q.data.size(); ++i) {
            res(i) = q.data[i];
        }
        return res;
    }

    caros_common_msgs::Q toRos(const rw::math::Q& q)
    {
        caros_common_msgs::Q res;
        res.data.resize(q.size());
        for (size_t i = 0; i < q.size(); ++i) {
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

    rw::math::Transform3D<> toRw(const geometry_msgs::Transform& transform) {
        rw::math::Vector3D<> p(transform.translation.x, transform.translation.y, transform.translation.z);
        rw::math::Quaternion<> q(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
        return rw::math::Transform3D<>(p, q);
    }

    rw::math::Transform3D<> toRw(const geometry_msgs::Pose& transform) {
        rw::math::Vector3D<> p(transform.position.x, transform.position.y, transform.position.z);
        rw::math::Quaternion<> q(transform.orientation.x, transform.orientation.y, transform.orientation.z, transform.orientation.w);
        return rw::math::Transform3D<>(p, q);
    }

    rw::math::Wrench6D<> toRw(const geometry_msgs::Wrench& wrench) {
        rw::math::Wrench6D<> w(wrench.force.x, wrench.force.y, wrench.force.z, wrench.torque.x, wrench.torque.y, wrench.torque.z);
        return w;
    }

    geometry_msgs::Wrench toRos(const rw::math::Wrench6D<>& w) {
        geometry_msgs::Wrench wrench;
        wrench.force.x = w.force()[0];
        wrench.force.y = w.force()[1];
        wrench.force.z = w.force()[2];
        wrench.torque.x = w.torque()[0];
        wrench.torque.y = w.torque()[1];
        wrench.torque.z = w.torque()[2];
        return wrench;
    }

    rw::math::VelocityScrew6D<> toRw(const geometry_msgs::Twist& twist) {
        rw::math::VelocityScrew6D<> vs(twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z);
        return vs;
    }

    geometry_msgs::Twist toRos(const rw::math::VelocityScrew6D<>& vs) {
        geometry_msgs::Twist twist;
        twist.linear.x = vs(0);
        twist.linear.y = vs(1);
        twist.linear.z = vs(2);
        twist.angular.x = vs(3);
        twist.angular.y = vs(4);
        twist.angular.z = vs(5);
        return twist;
    }

    float toRos(const float value) {
        return value;
    }

    bool toRos(const bool value) {
        return value;
    }

    rw::models::WorkCell::Ptr getWorkCell() {
        return getWorkCell("/caros/workcell");
    }

    rw::models::WorkCell::Ptr getWorkCell(const std::string& paramname) {
        ros::NodeHandle node("~");
        std::string workcellFile;
        bool paramFound;
        paramFound = node.getParam(paramname, workcellFile);
        if (!paramFound) {
            ROS_ERROR_STREAM("No such parameter on the parameter server: " << paramname);
            return NULL;
        } else if (workcellFile.empty()){
            ROS_ERROR_STREAM("The value of the parameter is empty!");
            return NULL;
        }
        ROS_DEBUG_STREAM("loading file: " << workcellFile );
        /* TODO:
         * The following load() function could potentially throw an exception, that should probably be caught here to make the code using this function not care about catching the exception and handle it nicely - throw it into a ROS_WARN or similar.
         * ^- "An exception is thrown if the file can't be loaded." : http://www.robwork.dk/apidoc/nightly/rw/classrw_1_1loaders_1_1WorkCellLoader_1_1Factory.html
         */
        rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellFactory::load(workcellFile);
        return wc;
    }

} // namespace
