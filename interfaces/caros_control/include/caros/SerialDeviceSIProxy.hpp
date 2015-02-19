#ifndef SerialDeviceSIProxy_HPP_
#define SerialDeviceSIProxy_HPP_

#include <caros_control_msgs/RobotState.h>

#include <rw/math.hpp>

#include <ros/ros.h>

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
