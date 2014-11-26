#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

pub_joint_states = rospy.Publisher('joint_states', JointState)
joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint','wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

def talker():
	rospy.init_node('talker', anonymous=True)
	r = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		msg = JointState()
		msg.header.stamp = rospy.get_rostime()
		msg.header.frame_id = "Dummy impl"
		msg.name = joint_names
		msg.position = [0.0] * 6
		#for i, jd in enumerate(state.joint_data):
		#	msg.position[i] = jd.q_actual + joint_offsets.get(joint_names[i], 0.0)
		msg.velocity = [0]*6
		msg.effort = [0]*6
		pub_joint_states.publish(msg)
		r.sleep()
 
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass