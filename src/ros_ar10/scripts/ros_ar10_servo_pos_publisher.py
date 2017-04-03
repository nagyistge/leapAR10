#!/usr/bin/env python

import sys
import os
import argparse
import rospy
from ros_ar10_class import ar10
from std_msgs.msg import UInt16MultiArray

def publisher(is_left):
	topic_name = 'servo_positions'
	node_name = 'ros_ar10_servo_pos_publisher'
	if is_left:
		topic_name = topic_name + ".left"
		node_name = node_name + ".left"
	else:
		topic_name = topic_name + ".right"
		node_name = node_name + ".right"
	pub = rospy.Publisher(topic_name, UInt16MultiArray, queue_size=10)
	rospy.init_node(node_name, anonymous=False)
	rate = rospy.Rate(1) # 1hz

	while not rospy.is_shutdown():
		values = UInt16MultiArray()
		for joint in range (0, 10): # for all servos ...
			values.data.insert(joint, hand.get_position(joint)) # get servo position
			rospy.loginfo('servo %d = %d ' , joint, values.data[joint])
		pub.publish(values) # publish servo position
		rate.sleep()

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('-d', '--device', dest="device", required=True, help="serial device")
	hand_group = parser.add_mutually_exclusive_group(required=True)
	hand_group.add_argument('-l', '--left', dest="left", help = "use left hand",action='store_true')
	hand_group.add_argument('-r', '--right', dest="right", help = "use right hand",action='store_true')
	args = parser.parse_args()
	file_path = os.path.dirname(os.path.realpath(__file__)) + "/ros_ar10_calibration_file"
	if args.left:
		file_path = file_path + ".left"
	if args.right:
		file_path = file_path + ".right"
	file_path = file_path
	device_path = args.device
	hand = ar10(device_path, file_path)
	try:
		publisher(args.left)
	except rospy.ROSInterruptException:
		pass
