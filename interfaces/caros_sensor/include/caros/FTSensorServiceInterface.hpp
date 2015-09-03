/**/
#ifndef CAROS_FTSENSORSERVICEINTERFACE_HPP
#define CAROS_FTSENSORSERVICEINTERFACE_HPP

#include <rw/math/Wrench6D.hpp>

#include <ros/ros.h>

/**
 * @brief describe the minimum interface of a Force/Torque sensing device.
 */
class FTSensorServiceInterface {
public:
    typedef rw::common::Ptr<FTSensorServiceInterface> Ptr;

    //! constructor
	FTSensorServiceInterface(const std::string& service_name);

	FTSensorServiceInterface(rw::common::Ptr<ros::NodeHandle> nh);

	virtual ~FTSensorServiceInterface(){}

	void publish(const rw::math::Wrench6D<>& wrench, const std::string& refframe);

protected:
	rw::common::Ptr<ros::NodeHandle> _nodeHnd;

private:
    ros::Publisher _wrenchDataPublisher;

};

#endif
