/**/
#ifndef CAROS_POSESENSORSERVICEINTERFACE_HPP
#define CAROS_POSESENSORSERVICEINTERFACE_HPP

#include <rw/math/Transform3D.hpp>

#include <ros/ros.h>

#include <memory>
#include <vector>

#define POSE_SENSOR_POSE_PUBLISHER_QUEUE_SIZE 1
#define POSE_SENSOR_SERVICE_INTERFACE_SUB_NAMESPACE "caros_pose_sensor_service_interface"

namespace caros
{
/**
 * @brief standard interface for pose sensor that can track a number of
 *  poses.
 */
class PoseSensorServiceInterface
{
 public:
  //! pointer type
  typedef std::shared_ptr<PoseSensorServiceInterface> Ptr;

  //! constructor
  PoseSensorServiceInterface(const ros::NodeHandle& nodehandle);

 protected:
  //! initialize ros interface
  bool configureInterface();

  //! publish poses read by sensor
  void publishPoses(const std::vector<rw::math::Transform3D<>>& poses, const std::vector<int>& ids,
                    const std::vector<float>& qualities);

 private:
  PoseSensorServiceInterface(){};

  ros::NodeHandle nodehandle_;
  ros::Publisher pose_publisher_;
};
}

#endif
