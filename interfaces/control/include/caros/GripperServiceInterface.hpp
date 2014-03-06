#ifndef GRIPPERSERVICEINTERFACE_HPP_
#define GRIPPERSERVICEINTERFACE_HPP_

#include <rw/math/Q.hpp>
#include <rw/common/Ptr.hpp>

#include <ros/ros.h>
#include <caros_control/GripperMoveQ.h>
#include <caros_control/GripperGripQ.h>
#include <caros_control/GripperSetForceQ.h>
#include <caros_control/GripperSetVelocityQ.h>
#include <caros_control/GripperStopMovement.h>

#include <string>

namespace caros {
  /**
   * @brief This is the gripper interface. It defines the
   * minimum interface that a configuration based robotic grasping device needs
   * to implement.
   *
   * In ROS the namespace of the node is used and it is important that
   * not two GripperServiceInterfaces are running in the same namespace.
   *
   */
  class GripperServiceInterface {
  public:

    /**
     * @brief constructor.
     * @param nodehandle [in] the nodehandle to use for services.
     */
    GripperServiceInterface(rw::common::Ptr<ros::NodeHandle> nodehandle);

    /**
     * @brief constructor
     * @param service_name [in] the namespace used for creating a nodehandle.
     */
    GripperServiceInterface(const std::string& service_name);

    /**
     * @brief virtual destructor
     */
    virtual ~GripperServiceInterface();


    bool configureGripperService();
    bool cleanupGripperService();

    /**
     * @brief signal the gripper to move into a specific configuration Q.
     * @param q
     * @return
     */
    virtual bool moveQ(const rw::math::Q& q) = 0;

    /**
     * @brief signal the gripper to move into a specific configuration Q. The gripper will not show an error in its state if the configuration Q can not be reached.
     * @param q
     * @return
     */
    virtual bool gripQ(const rw::math::Q& q) = 0;

    /**
     * @brief set the desired force configuration that the gripper should use.
     * @param q
     * @return
     */
    virtual bool setForceQ(const rw::math::Q& q) = 0;

    /**
     * @brief set the desired velocity configuration that the gripper should use.
     * @param q
     * @return
     */
    virtual bool setVelocityQ(const rw::math::Q& q) = 0;

    /**
     * @brief signal the gripper to stop all its movements.
     * It should not power down the gripper and/or disconnect from the gripper.
     * @return
     */
    virtual bool stopMovement(void) = 0;

  protected:

    /**
     * @brief publish the state of the gripper. Uses GripperState messages
     * @param q [in] joint configuration
     * @param dq [in] joint velocity
     */
    void publishState(const rw::math::Q& q,
                        const rw::math::Q& dq,
                        const rw::math::Q& jointforce,
                        bool isMoving,
                        bool isBlocked,
                        bool isStopped,
                        bool isEstopped);


  private:
    /**
     * @brief private default constructor.
     * This is declared as private to enforce deriving classes to call an available public constructor and enforce that the ROS services are properly initialised.
     */
    GripperServiceInterface();

    /**
     * @brief internal GripperServiceInterface function to properly advertise the services using ROS and setup the callbacks.
     * This should not be called directly.
     */
    void initGripperService();


    /* - these functions should be grouped together in the documentation t shortly describe that these are converting from ROS types to e.g. RobWork types according to what the interface expects - */
    bool moveQHandle(caros_control::GripperMoveQ::Request& request,
		     caros_control::GripperMoveQ::Response& response);

    bool gripQHandle(caros_control::GripperGripQ::Request& request,
		     caros_control::GripperGripQ::Response& response);

    bool setForceQHandle(caros_control::GripperSetForceQ::Request& request,
			 caros_control::GripperSetForceQ::Response& response);

    bool setVelocityQHandle(caros_control::GripperSetVelocityQ::Request& request,
			    caros_control::GripperSetVelocityQ::Response& response);

    bool stopMovementHandle(caros_control::GripperStopMovement::Request& request,
			    caros_control::GripperStopMovement::Response& response);

  private:
    rw::common::Ptr<ros::NodeHandle> _nodeHnd;


    ros::Publisher _gripperStatePublisher;

    ros::ServiceServer _srvMoveQ;
    ros::ServiceServer _srvGripQ;
    ros::ServiceServer _srvSetForceQ;
    ros::ServiceServer _srvSetVelocityQ;
    ros::ServiceServer _srvStopMovement;

    std::string _gripperName;

    bool _initialized;

  };
} // namespace
#endif
