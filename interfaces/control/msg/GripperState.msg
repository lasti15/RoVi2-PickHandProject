# GripperState represents the state of one a single gripper in the setup.

# Header containing information about time and frameid
Header header

# Joint configuration. 
# Angles should be represented as radians and distances in meters.
caros_common/Q q

# Velocities should be represented as radians per sec 
caros_common/Q dq

# Approximate force applied by the gripper joints (most implementations simply output the measured current)
caros_common/Q force

# Is the gripper trying to move to a target (notice it can still be blocked even if isMoving is true) 
bool isMoving

# If the gripper is moving then it may be blocked.
bool isBlocked

# If the gripper reached its target then isStopped is true
bool isStopped

# Is Emergency Stopped
bool estopped

