#ifndef CAROS_FTSENSORSERVICEINTERFACE_HPP
#define CAROS_FTSENSORSERVICEINTERFACE_HPP

#include <rw/math/Wrench6D.hpp>

#include <ros/ros.h>

#include <memory>
#include <string>

#define FT_SENSOR_WRENCH_PUBLISHER_QUEUE_SIZE 1
#define FT_SENSOR_SERVICE_INTERFACE_SUB_NAMESPACE "caros_ft_sensor_service_interface"

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
  FTSensorServiceInterface(ros::NodeHandle nodehandle);

  //! destructor
  virtual ~FTSensorServiceInterface()
  {
  }

 protected:
  //! initialize ros interface
  bool configureInterface();

  //! send the current F/T reading
  void publish(const rw::math::Wrench6D<>& wrench, const std::string& refframe);

 private:
  FTSensorServiceInterface(){};

  ros::NodeHandle nodehandle_;
  ros::Publisher wrench_data_publisher_;
};
}

#endif
