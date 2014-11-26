/**/
#ifndef KINECTSENSORSERVICEINTERFACE_HPP
#define KINECTSENSORSERVICEINTERFACE_HPP

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>


#include "ros/ros.h"
#include "marvin_common/SDHMoveQ.h"
#include "marvin_common/SDHGripQ.h"
#include "marvin_common/SDHStop.h"
#include "marvin_common/SDHPause.h"

#include <string>

/**
 * @brief describe the minimum interface of a Force/Torque sensing device.
 */
class KinectSensorServiceInterface {
public:
	KinectSensorServiceInterface(const std::string& service_name);

protected:
	rw::common::Ptr<ros::NodeHandle> _nodeHnd;

private:
    //ros::Publisher _FTSensorStatePublisher;
    //ros::ServiceServer _srvMoveQ;

};

#endif //#ifndef SDHSERVICEINTERFACE_HPP
