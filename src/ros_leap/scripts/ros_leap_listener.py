#!/usr/bin/env python

import sys
import signal
import rospy
import Leap
from ros_leap.msg import Leap_Frame, Hand, Finger, Bone, Arm, Vector3


def listener():
	finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
	bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']

	def callback(leap_msg):

		rospy.loginfo(leap_msg)
		rospy.loginfo("A Leap Frame received")
		# end of callback

	rospy.init_node(node_name, anonymous=True)
	rospy.Subscriber(topic_name, Leap_Frame, callback)
	rospy.spin()

if __name__ == "__main__":
	# store the original SIGINT handler
	topic_name = 'leap_frame'
	node_name = 'ros_leap_listener'
	listener()