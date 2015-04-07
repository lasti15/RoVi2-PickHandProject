#ifndef CAROS_SDH_SDH_NODE_H
#define CAROS_SDH_SDH_NODE_H

#include <caros/caros_node_service_interface.h>
#include <caros/gripper_service_interface.h>

#include <rw/math/Q.hpp>
#include <rw/common/Timer.hpp>
#include <rw/math/Constants.hpp>

#include <rwhw/sdh/SDHDriver.hpp>

#include <ros/ros.h>

#include <string>

#define MIN_TIME_THRESHOLD_FOR_CALCULATING_VELOCITY 0.1
#define MOVE_DISTANCE_STOPPED_THRESHOLD 0.01
#define MAX_TIME_WAITING_FOR_MOVE_TO_FINISH_BEFORE_INTERVENING 2.0
#define MINIMUM_VELOCITY_BEFORE_CONSIDERED_NOT_MOVING (5.0 * rw::math::Deg2Rad)

#define SUPPORTED_Q_LENGTH_FOR_SDHNODE 7

/**
 * @brief ROS node for controlling SDH.
 */
class SDHNode: public caros::CarosNodeServiceInterface, public caros::GripperServiceInterface {
public:
    //! constructor
    SDHNode(const ros::NodeHandle& nodehandle);

    //! destructor
    virtual ~SDHNode();

    //! @copydoc caros::GripperServiceInterface::moveQ
    bool moveQ(const rw::math::Q& q);
    //! @copydoc caros::GripperServiceInterface::moveQ
    bool gripQ(const rw::math::Q& q);
    //! @copydoc caros::GripperServiceInterface::setForceQ
    bool setForceQ(const rw::math::Q& q);
    //! @copydoc caros::GripperServiceInterface::setVelocityQ
    bool setVelocityQ(const rw::math::Q& q);
    //! @copydoc caros::GripperServiceInterface::stopMovement
    bool stopMovement();

    /* TODO: Properly document the error codes */
    /* TODO: Consider better error codes for SDHNODE_INTERNAL_ERROR */
    /* The enum order should not be changed, as recorded ROS sessions would then be invalidated */
    enum SDHNODE_ERRORCODE { SDHNODE_SDH_DEVICE_ALREADY_ACTIVE = 1, SDHNODE_CAROS_GRIPPER_SERVICE_CONFIGURE_FAIL, SDHNODE_UNSUPPORTED_INTERFACE_TYPE, SDHNODE_SDH_DEVICE_CONNECT_FAILED, SDHNODE_INTERNAL_ERROR, SDHNODE_SDH_DEVICE_NO_CONNECTION, SDHNODE_NO_SDH_DEVICE, SDHNODE_UNSUPPORTED_Q_LENGTH };

protected:
    // hooks implemented from CarosNodeServiceInterface base class
    bool activateHook();
    bool recoverHook();

    void runLoopHook();
    void errorLoopHook();
    void fatalErrorLoopHook();

    // Utility functions to configure and connect to the SDH device
    bool configureSDHDevice();
    bool connectToSDHDevice();

private:
    bool verifyWorkingSDHDevice(const std::string& functionName);
    bool verifyQLength(const rw::math::Q& q, const std::string& functionName);

    ros::NodeHandle nodeHandle_;

    enum SDH_STATE { WAIT, MOVE_WAIT };
    SDH_STATE currentState_, nextState_;

    rw::common::Timer moveStartTimer_, velUpdateTimer_;
    rw::math::Q moveQ_, velQ_, currentQ_, lastQ_;

    rwhw::SDHDriver *sdh_;

    /* Variables that are to be fetched from a ROS parameter server */
    std::string interfaceType_;
    std::string rs232Device_;
    int rs232Port_;
    int rs232BaudRate_;
    double rs232Timeout_;
    std::string canDevice_;
    int canBaudRate_;
    double canTimeout_;
};

#endif /* include guard */
