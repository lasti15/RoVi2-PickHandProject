/**/
#ifndef CAROS_ROBOTIQ3NODE_HPP
#define CAROS_ROBOTIQ3NODE_HPP

#include <caros/caros_node_service_interface.h>
#include <caros/GripperServiceInterface.hpp>

#include <rwhw/robotiq/Robotiq3.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/math/Q.hpp>

#include <ros/ros.h>

/**
 * @brief Ros node for controlling Robotiq-3 gripper.
 */
class Robotiq3Node: public caros::CarosNodeServiceInterface, public caros::GripperServiceInterface{
public:

    enum ERRORCODE {
        CONNECTION_ERROR = 1 //! Connection to robotiq hand was not possible
    };

    //! constructor
    Robotiq3Node(const ros::NodeHandle& nodehandle);

    //! destructor
    virtual ~Robotiq3Node();

    //! @copydoc caros::GripperServiceInterface::moveQ
    bool moveQ(const rw::math::Q& q);

    //! @copydoc caros::GripperServiceInterface::moveQ
    bool gripQ(const rw::math::Q& q);

    //! @copydoc caros::GripperServiceInterface::setForceQ
    bool setForceQ(const rw::math::Q& q);

    //! @copydoc caros::GripperServiceInterface::setVelocityQ
    bool setVelocityQ(const rw::math::Q& q);

    //! @copydoc caros::GripperServiceInterface::stopMovement
    bool stopMovement(void);

    /* TODO: Properly document the error codes */
    /* TODO: Consider better error codes for ROBOTIQ3NODE_INTERNAL_ERROR */
    /* The enum order should not be changed, as recorded ROS sessions would then be invalidated */
    enum ROBOTIQ3NODE_ERRORCODE { ROBOTIQ3NODE_ROBOTIQ_DEVICE_ALREADY_ACTIVE = 1, ROBOTIQ3NODE_CAROS_GRIPPER_SERVICE_CONFIGURE_FAIL, ROBOTIQ3NODE_UNSUPPORTED_INTERFACE_TYPE, ROBOTIQ3NODE_ROBOTIQ_DEVICE_CONNECT_FAILED, ROBOTIQ3NODE_INTERNAL_ERROR, ROBOTIQ3NODE_ROBOTIQ_DEVICE_NO_CONNECTION, ROBOTIQ3NODE_NO_ROBOTIQ_DEVICE, ROBOTIQ3NODE_UNSUPPORTED_Q_LENGTH };


protected:
    // hooks implemented from CarosNodeServiceInterface base class
    bool activateHook();
    bool recoverHook();

    void runLoopHook();
    void errorLoopHook();
    void fatalErrorLoopHook();

    // Utility functions to configure and connect to the Robotiq device
    bool configureRobotiqDevice();
    bool connectToRobotiqDevice();


protected:
    typedef enum{MOVE,GRIP,STOP} CmdType;
    int _lastCmd;
    ros::Time _lastLoopTime;
    rw::math::Q _lastQ;
    std::string _ip;
    int _port;

private:
    rw::common::Ptr<rwhw::Robotiq3> _robotiq;
    ros::NodeHandle _nodeHandle;
};


#endif //#ifndef CAROS_ROBOTIQ3NODE_HPP
