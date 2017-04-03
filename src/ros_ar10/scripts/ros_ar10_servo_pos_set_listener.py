#!/usr/bin/env python

import sys
import os
import argparse
import rospy
from ros_ar10_class import ar10
from std_msgs.msg import UInt16MultiArray

def listener():

	def callback(values):
		for joint in range (0, 10): # for all servos ...
			rospy.loginfo('servo %d = %d ' , joint, values.data[joint])
			hand.move(joint, values.data[joint])

	rospy.init_node(node_name, anonymous=False)
	rospy.Subscriber(topic_name, UInt16MultiArray, callback)
	rospy.spin()

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('-d', '--device', dest="device", required=True, help="serial device")
	hand_group = parser.add_mutually_exclusive_group(required=True)
	hand_group.add_argument('-l', '--left', dest="left", help = "use left hand",action='store_true')
	hand_group.add_argument('-r', '--right', dest="right", help = "use right hand",action='store_true')
	args = parser.parse_args()

	topic_name = 'set_servo_positions'
	node_name = 'ros_ar10_servo_pos_set_listener'
	if args.left:
		topic_name = topic_name + ".left"
		node_name = node_name + ".left"
	else:
		topic_name = topic_name + ".right"
		node_name = node_name + ".right"
	args = parser.parse_args()
	file_path = os.path.dirname(os.path.realpath(__file__)) + "/ros_ar10_calibration_file"
	if args.left: #if conditions relating to corresponding arguments
		file_path = file_path + ".left"
	if args.right: #if conditions relating to corresponding arguments
		file_path = file_path + ".right"
	file_path = file_path
	device_path = args.device
	hand = ar10(device_path, file_path)
	listener()