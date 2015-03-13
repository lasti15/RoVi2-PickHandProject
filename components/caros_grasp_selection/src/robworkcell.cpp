/**
 * @brief the robwork cell maps the actual real world state information to
 * a robwork workcell. The user may decide which parts to map to what.
 */

#include "ros/ros.h"
#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/Device.hpp>
//#include <caros_grasp_selection/SelectGrasps.hpp>
#include "GraspDBRepository.hpp"
#include <caros/common.h>

#include <caros_grasp_selection/SelectGrasps.h>
#include <caros_common_msgs/GetRWState.h>

#include <rwlibs/task.hpp>

#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"

#include <boost/algorithm/string.hpp>

using namespace rw::models;
using namespace rw::common;
using namespace rw::kinematics;

// global variables
WorkCell::Ptr workcell;

void arrayCallback(const std_msgs::Float64MultiArray::ConstPtr& array);

class RobotStateListener {
public:
	RobotStateListener(Device::Ptr dev, State::Ptr state):
		_dev(dev), _state(state)
	{
	}

	void callbackJointState(const sensor_msgs::JointState::ConstPtr& jstate){
		rw::math::Q q = _dev->getQ(*_state);
		if( q.size() != jstate->position.size()){
			ROS_ERROR_STREAM("Mismatching dimensions to robot state information: " << q.size() << "!=" << jstate->position.size());
			return;
		}
		for(int i=0;i<jstate->position.size();i++){
			q(i) = jstate->position[i];
		}
		_dev->setQ(q,*_state);
	}

	void callback64(const std_msgs::Float64MultiArray::ConstPtr& array){

		rw::math::Q q = _dev->getQ(*_state);
		if( q.size() != array->data.size()){
			ROS_ERROR_STREAM("Mismatching dimensions to robot state information: " << q.size() << "!=" << array->data.size());
			return;
		}
		for(int i=0;i<array->data.size();i++){
			q(i) = array->data[i];
		}
		_dev->setQ(q,*_state);
	}

	void callback32(const std_msgs::Float32MultiArray::ConstPtr& array){
		rw::math::Q q = _dev->getQ(*_state);
		if( q.size() != array->data.size()){
			ROS_ERROR_STREAM("Mismatching dimensions to robot state information: " << q.size() << "!=" << array->data.size());
			return;
		}
		for(int i=0;i<array->data.size();i++){
			q(i) = array->data[i];
		}
		_dev->setQ(q,*_state);
	}

public:
	Device::Ptr _dev;
	State::Ptr _state;
};

State::Ptr stateptr;
bool rwstatecallback(caros_common_msgs::GetRWState::Request& request, caros_common_msgs::GetRWState::Response& response){
	response.state = caros::toRos( *stateptr );
	return true;
}

// main initialization
int main(int argc, char **argv)
{
	// initialize ros
	ros::init(argc, argv, "RobWorkCell");

	// initialize the node handle
    ros::NodeHandle nh("~");

	// initialize workspace information, we need to know which gripper to use
	// and its kinematic/geometric model description
    workcell = caros::getWorkCell();
	if( workcell==NULL ){
	    ROS_ERROR_STREAM("Unable to get workcell ");
	}

	// test get topics
	std::vector<ros::master::TopicInfo> topics;

	ros::master::getTopics(topics);
	std::cout << "NR topics: " << topics.size() << std::endl;
	BOOST_FOREACH(ros::master::TopicInfo info,topics){
		std::cout << info.name << " " << info.datatype << std::endl;
	}



	// now for each device in the workcell, check if there are any parameters specifying
	// configurations.
	std::vector<ros::Subscriber> subs;
	std::vector<RobotStateListener*> listeners;
	stateptr = ownedPtr( new State(workcell->getDefaultState()) );
	BOOST_FOREACH(Device::Ptr dev, workcell->getDevices()){
		std::string topic;

		std::string devname = dev->getName();
		boost::replace_all(devname,"-","_");
		nh.param(devname, topic, std::string(""));
		if( topic.empty() ){
			std::cout << "NO TOPIC FOR device: " << devname << std::endl;
			continue;
		}

		// check type of topic
		std::vector<ros::master::TopicInfo> topics1;
		std::string topictype;
		ros::master::getTopics(topics1);
		BOOST_FOREACH(ros::master::TopicInfo info,topics1){
			if(info.name == topic){
				topictype = info.datatype;
			}
		}
		if(topictype.empty()){
			ROS_ERROR_STREAM("Could not resolve topic type!");
			continue;
		}
		ROS_INFO_STREAM("Subscribing to: " << topic);
		std::cout << "Subscribing to: " << topic << " with type " << topictype << " For device: " << devname << std::endl;
		// else try setting up a mapping from topic to changes in state
		listeners.push_back( new RobotStateListener(dev, stateptr) );

		if(topictype=="sensor_msgs/JointState"){
			subs.push_back( nh.subscribe(topic, 1, &RobotStateListener::callbackJointState, listeners.back()) );
		} else if( topictype=="std_msgs/Float64MultiArray"){
			subs.push_back( nh.subscribe(topic, 1, &RobotStateListener::callback64, listeners.back()) );
		} else if( topictype=="std_msgs/Float32MultiArray"){
			subs.push_back( nh.subscribe(topic, 1, &RobotStateListener::callback32, listeners.back()) );
		} else {
			ROS_ERROR_STREAM("Unsupported topic msg type: " << topictype);
		}
	}

	// advertise the get state service
	ros::ServiceServer rwstateservice = nh.advertiseService("/caros/getworkcellstate", &rwstatecallback );

	ros::spin();

	return 0;
}
