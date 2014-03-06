#include <caros/Robotiq3Node.hpp>

#include <rw/rw.hpp>

USE_ROBWORK_NAMESPACE

using namespace robwork;


Robotiq3Node::Robotiq3Node(const std::string& name):
        caros::CarosNodeServiceInterface(name),
        caros::GripperServiceInterface(name),
	_lastQ(4,0,0,0,0),
	_ip("192.168.100.21"),
	_port(502),
	_nodeName(name)
{
    configureNode();
}

void Robotiq3Node::configureHook(){
    ros::NodeHandle nh(_nodeName);
    nh.param("ip", _ip, std::string("192.168.100.21") );
    nh.param("port", _port, 502);
    _robotiq = ownedPtr( new rwhw::Robotiq3() );

    if(!configureGripperService() )
        RW_THROW("Could not configure gripper service!");
}

void Robotiq3Node::startHook(){
    // when starting the node we connect to the hand
    if( !_robotiq->connect(_ip,_port) ){
        RW_THROW("Could not connect to robotiq hand using: " << _ip << " : " << _port);
    }
}

void Robotiq3Node::stopHook(){
    // when starting the node we connect to the hand
    _robotiq->disconnect();
}

void Robotiq3Node::recoverHook(){
    // weeeeeell depends on the error...
}

void Robotiq3Node::cleanupHook(){
    // destruct the robotiq driver
    _robotiq = NULL;
    cleanupGripperService();
}

bool Robotiq3Node::moveQ(const rw::math::Q& q){
    _robotiq->moveCmd(q);
    _lastCmd = MOVE;
    return true;
}

bool Robotiq3Node::gripQ(const rw::math::Q& q){
    _robotiq->moveCmd(q);
    _lastCmd = GRIP;
    return true;
}

bool Robotiq3Node::setForceQ(const rw::math::Q& q){
    _robotiq->setTargetQCurrent(q);
    return true;
}

bool Robotiq3Node::setVelocityQ(const rw::math::Q& q){
    _robotiq->setTargetQVel(q);
    return true;
}

bool Robotiq3Node::stopMovement(void){
    if(!isInRunning())
        return false;
    _robotiq->stopCmd();
    return true;
}


void Robotiq3Node::runloopHook() {
    ros::Time now = ros::Time::now();
    ros::Duration diff = now-_lastLoopTime;

    // publish state of gripper
    _robotiq->getAllStatus();
    Q q = _robotiq->getQ();
    Q dq = (q-_lastQ)/diff.toSec();
    Q force = _robotiq->getQCurrent();
    bool ismov = _robotiq->isGripperMoving();
    bool isblock = _robotiq->isGripperBlocked();
    bool isstop = _robotiq->isGripperStopped();
    bool isEstop = false;
    publishState(q,dq,force,ismov,isblock,isstop,isEstop);

    _lastQ = q;
    _lastLoopTime = now;
}

