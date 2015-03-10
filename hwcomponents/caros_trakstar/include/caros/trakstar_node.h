/**/
#ifndef CAROS_TRAKSTAR_NODE_H
#define CAROS_TRAKSTAR_NODE_H

#include <caros/caros_node_service_interface.h>
#include <caros/button_sensor_service_interface.h>
#include <caros/pose_sensor_service_interface.h>

#include "nodelet/nodelet.h"

#include <boost/thread/thread.hpp>

#include <queue>

namespace caros
{

/**
 * @brief Node to interface to the 6D trakstar pose sensor.
 */
class TrakstarNode : public nodelet::Nodelet,
                     public caros::CarosNodeServiceInterface,
                     public caros::ButtonSensorServiceInterface,
                     public caros::PoseSensorServiceInterface
{
public:
  //! constructor
  TrakstarNode();

  //! constructor
  TrakstarNode(const ros::NodeHandle& nodehandle);
  //! destructor
  virtual ~TrakstarNode();

  void onInit();

  enum TRAKSTAR_ERRORCODE { TRAKSTAR_POSESENSOR_CONFIGURE_FAIL = 1,
                            TRAKSTAR_BUTTONSERVICE_CONFIGURE_FAIL,
                            TRAKSTAR_DRIVER_INITIALIZATION_FAIL
  };

protected:
  /************************************************************************
   * Hooks implemented from CarosNodeServiceInterface base class
   ************************************************************************/
  //! @copydoc CarosNodeServiceInterface::activateHook
  bool activateHook();
  //! @copydoc CarosNodeServiceInterface::recoverHook
  bool recoverHook(){ return false;}
  //! @copydoc CarosNodeServiceInterface::runLoopHook
  void runLoopHook();
  //! @copydoc CarosNodeServiceInterface::errorLoopHook
  void errorLoopHook(){}
  //! @copydoc CarosNodeServiceInterface::fatalErrorLoopHook
  void fatalErrorLoopHook(){}

private:
  ros::NodeHandle nodehandle_;

  double max_pub_frequency_;
  std::string frame_id_, calibration_data_;

  class Trakstar *t_driver_;

  boost::shared_ptr<boost::thread> pubThread_; ///< The thread that reads and publishes data

};
}
#endif //#ifndef CAROS_TRAKSTAR_NODE_HPP
