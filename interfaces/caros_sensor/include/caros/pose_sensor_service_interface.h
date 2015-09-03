/**/
#ifndef CAROS_POSESENSORSERVICEINTERFACE_HPP
#define CAROS_POSESENSORSERVICEINTERFACE_HPP

#include <rw/math/Transform3D.hpp>

#include <ros/ros.h>

namespace caros
{

/**
 * @brief standard interface for pose sensor that can track a number of
 *  poses.
 */
class PoseSensorServiceInterface
{
public:
  //! smart pointer type
  typedef rw::common::Ptr<PoseSensorServiceInterface> Ptr;

  //! constructor
  PoseSensorServiceInterface(const ros::NodeHandle& nodehandle);

protected:
  //! initialize ros interface
  bool configureInterface();

  //! shutdown ros interface
  bool cleanupInterface();

  //! publish poses read by sensor
  void publishPoses(const std::vector<rw::math::Transform3D<> >& poses, const std::vector<int>& ids,
                    const std::vector<float>& qualities);

private:
  PoseSensorServiceInterface()
  {
  }
  ;

protected:
  ros::NodeHandle node_hnd_;

private:
  ros::Publisher pose_pub_;

};

}

#endif
