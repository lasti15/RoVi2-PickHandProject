#ifndef SerialDeviceSIProxy_HPP_
#define SerialDeviceSIProxy_HPP_

#include <caros_control_msgs/RobotState.h>

#include <rw/math.hpp>

#include <ros/ros.h>

/* TODO:
 * Could make the 'caros_control_msgs::RobotState _pRobotState' available to the user (as a copy), so if this SIP is being updated automatically in a thread, then it's not certain that a call to getTimeStamp and isMoving will be reading from the same robot state
 * Maybe even pack it into it's own structure with a c++/rw interface/types returned instead of the ROS types. The getQ, getQd, isMoving and getTimeStamp functions could be moved to become member functions of that struct/class.
 */

namespace caros {
/**
 * @brief this class implements a c++ proxy to control and read data from a SerialDeviceServiceInterface.
 */
    class SerialDeviceSIProxy {
    public:
        /**
         * @brief Constructor
         * @param[in] nodehandle
         * @param[in] devname The name of the CAROS serialdevice node
         */
        SerialDeviceSIProxy(ros::NodeHandle nodehandle, const std::string& devname);

        //! destructor
        virtual ~SerialDeviceSIProxy();

        //! @brief move robot in a linear Cartesian path
        bool moveLin(const rw::math::Transform3D<>& target, const float speed = 100, const float blend = 0);

        //! @brief move robot from point to point
        bool movePTP(const rw::math::Q& target, const float speed = 100, const float blend = 0);

        //! @brief move robot from point to point but using a pose as target (requires invkin)
        bool movePTP_T(const rw::math::Transform3D<>& target, const float speed = 100, const float blend = 0);

        //! @brief TODO Missing
        bool moveVelQ(const rw::math::Q& target);

        //! @brief TODO Missing
        bool moveVelT(const rw::math::VelocityScrew6D<>& target);

        //! @brief move robot in a servoing fasion using joint configurations
        /* There is no blend parameter, as it is irrelevent when doing servoing. */
        bool moveServoQ(const rw::math::Q& target, const float speed = 100);

        //! @brief move robot in a servoing fasion using pose configurations
        /* There is no blend parameter, as it is irrelevent when doing servoing. */
        bool moveServoT(const rw::math::Transform3D<>& target, const float speed = 100);

        //! @brief move robot with a hybrid position/force control
        bool moveLinFC(const rw::math::Transform3D<>& target,
                       const rw::math::Wrench6D<>& wtarget,
                       const float selection[6],
                       const std::string refframe,
                       const rw::math::Transform3D<> offset,
                       const float speed = 100,
                       const float blend = 0);

        //! @brief hard stop the robot,
        bool stop();

        //! @brief pause the robot, should be able to continue trajectory
        bool pause();

        //! @brief enable safe mode, so that robot stops when collisions are detected
        bool setSafeModeEnabled(const bool enable);

        /**
         * @brief Get the last reported joint configuration
         *
         * @returns the joint configuration (the type of values are very implementation specific, but as a guideline it's recommended to keep angles in radians, and distances in meters)
         *
         * @note Make sure to look at getTimeStamp for ensuring the sample is "current enough"
         */
        rw::math::Q getQ();

        /**
         * @brief Get the last reported velocities
         *
         * @returns the velocities (the type of values are implementation specific, but as a guideline it's recommended to represent velocitites as radians per sec)
         *
         * @note Make sure to look at getTimeStamp for ensuring the sample is "current enough"
         */
        rw::math::Q getQd();

        /**
         * @brief Get information on whether the device was moving as of the sample timestamp
         *
         * @returns a boolean indicating whether the device was moving as of the sample timestamp
         *
         * @note Make sure to look at getTimeStamp for ensuring the sample is "current enough"
         */
        bool isMoving();

        /**
         * @brief get the timestamp of the received data - use this to verify that the data is "current enough" and supposedly valid - in the case that no data has yet been reported from the device
         *
         * @returns the timestamp of the last reported robot state
         */
        ros::Time getTimeStamp();

    protected:
        ros::NodeHandle _nodehandle;
        ros::ServiceClient _servoService;

        // services
        ros::ServiceClient _srvStop;
        ros::ServiceClient _srvStart;
        ros::ServiceClient _srvPause;

        ros::ServiceClient _srvMovePTP;
        ros::ServiceClient _srvMovePTP_T;
        ros::ServiceClient _srvMoveLin;
        ros::ServiceClient _srvMoveLinFC;
        ros::ServiceClient _srvMoveVelQ;
        ros::ServiceClient _srvMoveVelT;
        ros::ServiceClient _srvMoveServoQ;
        ros::ServiceClient _srvMoveServoT;

        ros::ServiceClient _srvSetSafeModeEnabled;

        // states
        ros::Subscriber _subRobotState;

    private:
        void handleRobotState(const caros_control_msgs::RobotState& state);

        caros_control_msgs::RobotState _pRobotState;
    };
}

#endif //end include guard
