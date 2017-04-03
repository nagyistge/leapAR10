#!/usr/bin/env python

import time
import sys
import os
import argparse
from ros_ar10_class import ar10

def calibrate_joint(joint):
	n_points = 0

	sum_x  = 0.0
	sum_y  = 0.0
	sum_xy = 0.0
	sum_xx = 0.0

	if joint == 9:
		hand.move(8, 5500)

	for target in range(4500, 8000, 500):
		hand.move(joint, target)
		hand.wait_for_hand()
		time.sleep(2.0)

		position = hand.get_read_position(joint)

		n_points = n_points + 1

		sum_x  = sum_x + position
		sum_y  = sum_y + target
		sum_xy = sum_xy + (position * target)
		sum_xx = sum_xx + (position * position)

	slope = ((sum_x * sum_y) - (n_points * sum_xy)) / ((sum_x * sum_x) - (n_points * sum_xx))
	y_intercept = (sum_y - (slope * sum_x)) / n_points

	hand.move(joint, 7950)
	hand.wait_for_hand()

	if joint == 9:
		hand.move(8, 7950)
		hand.wait_for_hand()

	return y_intercept, slope

def main():
	# open calibration file
	cal_file = open(file_path, "w")
	hand.open_hand()

	for joint in range(0, 10):
		y_intercept, slope = calibrate_joint(joint)
		print "joint = " + str(joint) + " y intercept = " + str(y_intercept) + " slope = " + str(slope)
		cal_file.write(str(joint))
		cal_file.write("\t")
		cal_file.write(str(y_intercept))
		cal_file.write("\t")
		cal_file.write(str(slope))
		cal_file.write("\n")

	# close calibration file
	cal_file.close()

	# destroy hand object
	hand.close()

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument('-d', '--device', dest="device", required=True, help="serial device")
	hand_group = parser.add_mutually_exclusive_group(required=True)
	hand_group.add_argument('-l', '--left', dest="left", help = "calibrate left hand",action='store_true')
	hand_group.add_argument('-r', '--right', dest="right", help = "calibrate right hand",action='store_true')
	args = parser.parse_args()
	file_path = os.path.dirname(os.path.realpath(__file__)) + "/ros_ar10_calibration_file"
	if args.left: #if conditions relating to corresponding arguments
		file_path = file_path + ".left"
	if args.right: #if conditions relating to corresponding arguments
		file_path = file_path + ".right"
	file_path = file_path
	device_path = args.device
	hand = ar10(device_path, file_path)
	main()


