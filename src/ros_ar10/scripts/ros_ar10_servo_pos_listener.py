#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16MultiArray
import argparse

def listener():

	def callback(values):
		for joint in range (0, 10): # for all servos ...
			rospy.loginfo('servo %d = %d ' , joint, values.data[joint])

	rospy.init_node(node_name, anonymous=True)
	rospy.Subscriber(topic_name, UInt16MultiArray, callback)
	rospy.spin()

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	hand_group = parser.add_mutually_exclusive_group(required=True)
	hand_group.add_argument('-l', '--left', dest="left", help = "use left hand",action='store_true')
	hand_group.add_argument('-r', '--right', dest="right", help = "use right hand",action='store_true')
	args = parser.parse_args()

	topic_name = 'servo_positions'
	node_name = 'ros_ar10_servo_pos_listener'
	if args.left:
		topic_name = topic_name + ".left"
		node_name = node_name + ".left"
	else:
		topic_name = topic_name + ".right"
		node_name = node_name + ".right"
	listener()