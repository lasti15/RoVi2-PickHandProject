#RobotState represents the state of one a single robot in the setup.
#Currently RobotState is constrainted to 6 degrees of freedom devices

#Header containing information about time and frameid
Header header

#Joint configuration. 
# Angles should be represented as radians and distances in meters.
caros_common/Q q

# velocities should be represented as radians per sec 
caros_common/Q dq

#Is the robot moving
bool isMoving

#Is the robot in collision
bool isColliding

#Is Emergency Stopped
bool estopped

