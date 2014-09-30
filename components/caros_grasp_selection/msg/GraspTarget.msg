
# name of the gripper that this grasp target is minded for
string gripper_name

# name of the frame that is used to define the pose of the gripper
string tcp_frame_name

# pose of tcp
geometry_msgs/Pose tcp_pose

# configuration of gripper when approaching object
float64[] q_approach

# configuration of gripper in the grasp of the object
float64[] q_grasp

# max forces to apply in the grasp
float64[] q_tau
