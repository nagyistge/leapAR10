#!/usr/bin/env python

import signal
import sys
import os
import thread
import time
import math
import numpy as np
import Leap
import rospy
from ros_leap.msg import Leap_Frame, Hand, Finger, Bone, Arm, Vector3
from std_msgs.msg import UInt16MultiArray

def run_program():
	while True:
		time.sleep(1000)

def angle(vector_1, vector_2):
	dot_product = vector_1.x*vector_2.x + vector_1.y*vector_2.y + vector_1.z*vector_2.z
	# norm of both direction vector should be 1
	return math.acos(dot_product)

def saturation(data, angle_min, angle_max):
	data_tmp = np.arange(15,dtype='f').reshape((5,3))
	for i in xrange(5):
		for j in xrange(3):
			if data[i][j] < angle_min[i][j]:
				data_tmp[i][j] = angle_min[i][j]
			elif data[i][j] > angle_max[i][j]:
				data_tmp[i][j] = angle_max[i][j]
			else:
				data_tmp[i][j] = data[i][j]
	data_tmp[0][0] = 0
	return data_tmp

def normalize(data, angle_min, angle_max, signal_min, signal_max):
	signal_tmp = np.arange(15,dtype='int').reshape((5,3))
	for i in xrange(5):
		for j in xrange(3):
			if i == 0:
				tmp = (data[i][j] - angle_min[i][j])/(angle_max[i][j] - angle_min[i][j])*(6000 - signal_max) + signal_max
			else:
				tmp = (data[i][j] - angle_min[i][j])/(angle_max[i][j] - angle_min[i][j])*(signal_min - signal_max) + signal_max
			if math.isnan(tmp):
				tmp = 4500
			signal_tmp[i][j] = tmp
	# prevent thumb collide with index finger
	if signal_tmp[0][1] < 6000:
		signal_tmp[0][1] = 6000
	if signal_tmp[0][2] < 7000:
		signal_tmp[0][2] = 7000
	return signal_tmp

def signal_to_frame(signals):
	frame = UInt16MultiArray()
	frame.data.insert(8, signals[0][1])
	frame.data.insert(9, signals[0][2])
	frame.data.insert(0, signals[1][0])
	frame.data.insert(1, signals[1][1])
	frame.data.insert(2, signals[2][0])
	frame.data.insert(3, signals[2][1])
	frame.data.insert(4, signals[3][0])
	frame.data.insert(5, signals[3][1])
	frame.data.insert(6, signals[4][0])
	frame.data.insert(7, signals[4][1])
	return frame

def listener():

	def callback(leap_msg):

		for hand_msg in leap_msg.hands:
			if (not hand_msg.is_valid) or hand_msg.confidence<0:
				continue

			data = np.arange(15,dtype='f').reshape((5,3))
			data.fill(0)

			# Get fingers
			for finger_msg in hand_msg.fingers:

				# Get bones
				for b in range(0, 3):
					bone1 = finger_msg.bones[b]
					bone2 = finger_msg.bones[b+1]
					data[finger_msg.type][b] = angle(bone1.direction, bone2.direction)

			angle_min = 0
			angle_max = 0
			publisher = pub_left
			if hand_msg.is_left:
				angle_min = angle_left_min
				angle_max = angle_left_max
				publisher = pub_left
			elif hand_msg.is_right:
				angle_min = angle_right_min
				angle_max = angle_right_max
				publisher = pub_right
			else:
				# error, should never be here
				continue
			angles = saturation(data, angle_min, angle_max)
			signals = normalize(angles, angle_min, angle_max, 4500, 8000)
			frame = signal_to_frame(signals)
			print frame
			publisher.publish(frame)
		# end of callback

	rospy.Subscriber(leap_listener_topic_name, Leap_Frame, callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		dir_path = os.path.dirname(os.path.realpath(__file__))
		angle_right_max = np.loadtxt(dir_path + '/rlac_calibrate_right_max')
		angle_right_min = np.loadtxt(dir_path + '/rlac_calibrate_right_min')
		angle_left_min = np.loadtxt(dir_path + '/rlac_calibrate_left_min')
		angle_left_max = np.loadtxt(dir_path + '/rlac_calibrate_left_max')
	except Exception as e:
		print "Calibration file missing"
		print "Please run rlac_mapper_calibrate.py"
		exit(2)

	leap_listener_topic_name = 'leap_frame'
	ar10_left_publisher_topic_name = 'set_servo_positions.left'
	ar10_right_publisher_topic_name = 'set_servo_positions.right'
	node_name = 'rlac_node'
	rospy.init_node(node_name, anonymous=False)
	pub_left = rospy.Publisher(ar10_left_publisher_topic_name, UInt16MultiArray, queue_size=10)
	pub_right = rospy.Publisher(ar10_right_publisher_topic_name, UInt16MultiArray, queue_size=10)
	listener()