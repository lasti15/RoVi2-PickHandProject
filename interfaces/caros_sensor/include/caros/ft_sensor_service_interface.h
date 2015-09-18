#ifndef CAROS_FTSENSORSERVICEINTERFACE_HPP
#define CAROS_FTSENSORSERVICEINTERFACE_HPP

#include <rw/math/Wrench6D.hpp>

#include <ros/ros.h>

#include <memory>
#include <string>

#define FT_SENSOR_WRENCH_PUBLISHER_QUEUE_SIZE 1

namespace caros
{

/**
 * @brief describe the minimum interface of a Force/Torque sensing device.
 */
class FTSensorServiceInterface
{
 public:
  //! pointer type
  typedef std::shared_ptr<FTSensorServiceInterface> Ptr;

  //! constructor
  FTSensorServiceInterface(ros::NodeHandle nh);

  //! destructor
  virtual ~FTSensorServiceInterface();

  //! send the current F/T reading
  void publish(const rw::math::Wrench6D<>& wrench, const std::string& refframe);

 protected:
  ros::NodeHandle nodeHnd_;

 private:
  ros::Publisher wrenchDataPublisher_;
};
}

#endif
