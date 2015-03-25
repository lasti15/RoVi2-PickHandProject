#ifndef SerialDeviceSIProxy_HPP_
#define SerialDeviceSIProxy_HPP_

#include <caros/exceptions.h>
#include <caros_control_msgs/RobotState.h>

#include <rw/math.hpp>

#include <ros/ros.h>

#include <stdexcept>

/* TODO:
 * Make the handleRobotState update/subscribed callback function be called asynchronously - see http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
 * Make sure to use c++11 mutex for synchronising the access to the variables that are updated asynchronously.
 *
 * ^- When having to supply the constructor with a ROS nodehandle, then it's up to the user to make a "proper" ros node and just use this wrapper as a convenience library for accessing SerialDeviceServices. It's not a complete standalone encapsulation of the ROS middleware layer, where the user don't need to know anything about ROS... That could probably be provided with one more layer on top of this SIProxy...
 *
 *
 * How to properly handle the cases where the _q, _dq, etc. variables havn't been updated yet?
 * ^- The user should look at the timestamp (i.e. getTimeStamp())
 * There could be possible inconsistency when some of the data is pulled from class variables and the timestamp is fetched from the _pRobotState.
 */

namespace caros {
/**
 * @brief this class implements a cpp proxy to control and read data from
 * a SerialDeviceServiceInterface.
 *
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

        /**
         * @brief move robot in a linear Cartesian path
         * @param[in] target The target to move to
         * @param[in] speed The movement speed (a value between 0 and 100 is expected)
         * @param[in] blend
         *
         * @returns a boolean indicating if the serial device accepted the command
         * @throws unavailableService when the command is currently unavailable. This indicates that the connection to the serial device is not fully working, or the serial device has not announced this service yet.
         * @throws badServiceCall when an error happened while communicating with the serial device.
         */
        bool moveLin(const rw::math::Transform3D<>& target, const float speed = 100.0f, const float blend = 0.0f);

        /**
         * @brief move robot from point to point
         * @param[in] target The target to move to
         * @param[in] speed The movement speed (a value between 0 and 100 is expected)
         * @param[in] blend
         *
         * @returns a boolean indicating if the serial device accepted the command
         * @throws unavailableService when the command is currently unavailable. This indicates that the connection to the serial device is not fully working, or the serial device has not announced this service yet.
         * @throws badServiceCall when an error happened while communicating with the serial device.
         */
        bool movePTP(const rw::math::Q& target, const float speed = 100.0f, const float blend = 0.0f);

        /**
         * @brief move robot from point to point but using a pose as target (requires invkin)
         * @param[in] target The target to move to
         * @param[in] speed The movement speed (a value between 0 and 100 is expected)
         * @param[in] blend
         *
         * @returns a boolean indicating if the serial device accepted the command
         * @throws unavailableService when the command is currently unavailable. This indicates that the connection to the serial device is not fully working, or the serial device has not announced this service yet.
         * @throws badServiceCall when an error happened while communicating with the serial device.
         */
        bool movePTP_T(const rw::math::Transform3D<>& target, const float speed = 100.0f, const float blend = 0.0f);

        /**
         * @brief move robot by some implementation specific distance based on the provided joint velocities
         * @param[in] target The joint velocities
         *
         * @returns a boolean indicating if the serial device accepted the command
         * @throws unavailableService when the command is currently unavailable. This indicates that the connection to the serial device is not fully working, or the serial device has not announced this service yet.
         * @throws badServiceCall when an error happened while communicating with the serial device.
         */
        bool moveVelQ(const rw::math::Q& target);

        /**
         * @brief move robot by some implementation specific distance based on the provided velocity screw
         * @param[in] target The velocity screw.
         *
         * @returns a boolean indicating if the serial device accepted the command
         * @throws unavailableService when the command is currently unavailable. This indicates that the connection to the serial device is not fully working, or the serial device has not announced this service yet.
         * @throws badServiceCall when an error happened while communicating with the serial device.
         */
        bool moveVelT(const rw::math::VelocityScrew6D<>& target);

        /**
         * @brief move robot in a servoing fashion using joint configurations
         * @param[in] target The joint configurations to move it
         * @param[in] speed The movement speed (a value between 0 and 100 is expected)
         *
         * @returns a boolean indicating if the serial device accepted the command
         * @throws unavailableService when the command is currently unavailable. This indicates that the connection to the serial device is not fully working, or the serial device has not announced this service yet.
         * @throws badServiceCall when an error happened while communicating with the serial device.
         */
        /* There is no blend parameter, as it is irrelevent when doing servoing. */
        bool moveServoQ(const rw::math::Q& target, const float speed = 100.0f);

        /**
         * @brief move robot in a servoing fasion using pose configurations
         * @param[in] target The pose configuration to move it
         * @param[in] speed The movement speed (a value between 0 and 100 is expected)
         *
         * @returns a boolean indicating if the serial device accepted the command
         * @throws unavailableService when the command is currently unavailable. This indicates that the connection to the serial device is not fully working, or the serial device has not announced this service yet.
         * @throws badServiceCall when an error happened while communicating with the serial device.
         */
        /* There is no blend parameter, as it is irrelevent when doing servoing. */
        bool moveServoT(const rw::math::Transform3D<>& target, const float speed = 100.0f);

        /* TODO:
         * Update the parameter description - especially for offset
         */
        /**
         * @brief move robot with a hybrid position/force control
         * @param[in] posTarget the target pose configuration
         * @param[in] offset ...
         * @param[in] wrenchTarget wrench
         * @param[in] controlGain Control gains for the force control. 0 means 100% position control, while values != 0 means force control with the numerical value.
         *
         * @returns a boolean indicating if the serial device accepted the command
         * @throws unavailableService when the command is currently unavailable. This indicates that the connection to the serial device is not fully working, or the serial device has not announced this service yet.
         * @throws badServiceCall when an error happened while communicating with the serial device.
         */
        bool moveLinFC(const rw::math::Transform3D<>& posTarget,
                       const rw::math::Transform3D<>& offset,
                       const rw::math::Wrench6D<>& wrenchTarget,
                       const rw::math::Q& controlGain);

        /**
         * @brief hard stop the robot
         *
         * @returns a boolean indicating if the serial device accepted the command
         * @throws unavailableService when the command is currently unavailable. This indicates that the connection to the serial device is not fully working, or the serial device has not announced this service yet.
         * @throws badServiceCall when an error happened while communicating with the serial device.
         */
        bool stop();

        /**
         * @brief pause the robot, should be able to continue trajectory
         *
         * @returns a boolean indicating if the serial device accepted the command
         * @throws unavailableService when the command is currently unavailable. This indicates that the connection to the serial device is not fully working, or the serial device has not announced this service yet.
         * @throws badServiceCall when an error happened while communicating with the serial device.
         */
        bool pause();

        /**
         * @brief enable safe mode, so that robot stops when collisions are detected
         * @param[in] enable a value of true enables safe mode, and false disables it
         *
         * @returns a boolean indicating if the serial device accepted the command
         * @throws unavailableService when the command is currently unavailable. This indicates that the connection to the serial device is not fully working, or the serial device has not announced this service yet.
         * @throws badServiceCall when an error happened while communicating with the serial device.
         */
        bool setSafeModeEnabled(const bool enable);

        rw::math::Q getQ();
        rw::math::Q getQd();

        ros::Time getTimeStamp();

        bool isMoving();

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
