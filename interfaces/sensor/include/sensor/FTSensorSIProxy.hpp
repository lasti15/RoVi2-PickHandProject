/*
 * FTSensorSIProxy.hpp
 *
 *  Created on: 15/05/2013
 *      Author: thomas
 */

#ifndef FTSENSORSIPROXY_HPP_
#define FTSENSORSIPROXY_HPP_

#include <marvin_common/WrenchData.h>

#include <rw/common/Ptr.hpp>
#include <rw/math.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>

/**
 * @brief this class implements a cpp proxy to control and read data from
 * a FTSensorServiceInterface.
 *
 */
class FTSensorSIProxy {
public:
	typedef rw::common::Ptr<FTSensorSIProxy> Ptr;

	//! constructor - create with device name
	FTSensorSIProxy(rw::common::Ptr<ros::NodeHandle> nhandle, const std::string& devname);

	//! destructor
	virtual ~FTSensorSIProxy();

	rw::math::Wrench6D<> getWrench();

	ros::Time getTimeStamp();

protected:
	rw::common::Ptr<ros::NodeHandle> _nodeHnd;

	// states
	ros::Subscriber _ftState;

private:
	boost::mutex _mutex;

	// state variables
	rw::math::Wrench6D<> _wrench;

	void handleFTState(const marvin_common::WrenchData& state);
	marvin_common::WrenchData _pFTState;
};

#endif /* FTSENSORSIPROXY_HPP_ */
