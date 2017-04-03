#!/usr/bin/env python

import time
import os
import argparse
from ros_ar10_class import ar10

def check_calibration(joint):
	n_points = 0

	sum_x  = 0.0
	sum_y  = 0.0
	sum_xy = 0.0
	sum_xx = 0.0

	if joint == 9:
		hand.move(8, 5500)

	print

	for target in range(4500, 8000, 500):
		hand.move(joint, target)
		hand.wait_for_hand()
		time.sleep(2.0)

		set_position = hand.get_set_position(joint)
		position     = hand.get_position(joint)
		error        = abs(set_position - position)

		print "joint = " + str(joint) + " set position = " + str(set_position) + " position = " + str(position) + " error = " + str(error)

	hand.move(joint, 7950)
	hand.wait_for_hand()

	if joint == 9:
		hand.move(8, 7950)
		hand.wait_for_hand()

	return

def main():
	hand.open_hand()
	hand.wait_for_hand()
	for joint in range(0, 10):
		check_calibration(joint)
	hand.close()

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument('-d', '--device', dest="device", required=True, help="serial device")
	hand_group = parser.add_mutually_exclusive_group(required=True)
	hand_group.add_argument('-l', '--left', dest="left", help = "calibrate left hand",action='store_true')
	hand_group.add_argument('-r', '--right', dest="right", help = "calibrate right hand",action='store_true')
	args = parser.parse_args()
	file_path = os.path.dirname(os.path.realpath(__file__)) + "/ros_ar10_calibration_file"
	if args.l:
		file_path = file_path + ".left"
	if args.r:
		file_path = file_path + ".right"
	print file_path
	device_path = args.device
	hand = ar10(device_path, file_path)
	main()


