#!/usr/bin/env python

import signal
import time
import sys
import os
import thread
import Leap
import numpy as np

step = 0;
last_step = 0;
counter = 0
angle = np.arange(15,dtype='f').reshape((5,3))
angle.fill(0)

def run_program():
	while True:
		time.sleep(1000)

def exit_gracefully(signum, frame):
	# restore the original signal handler as otherwise evil things will happen
	# in raw_input when CTRL+C is pressed, and our signal handler is not re-entrant
	signal.signal(signal.SIGINT, original_sigint)
	global step
	if step == 0:
		print "open LEFT hand over sensor and ctrl+c to start calibrate min signal"
		step =1;
		signal.signal(signal.SIGINT, exit_gracefully)
	elif step == 1:
		print "\nctrl+c to stop calibrate min signal"
		step = 2;
		signal.signal(signal.SIGINT, exit_gracefully)
	elif step == 2:
		print "\nclose LEFT hand over sensor and ctrl+c to start calibrate max signal"
		step = 3
		np.savetxt(dir_path+"/rlac_calibrate_left_min", angle);
		signal.signal(signal.SIGINT, exit_gracefully)
	elif step == 3:
		print "\nctrl+c to stop calibrate max signal"
		step = 4
		signal.signal(signal.SIGINT, exit_gracefully)
	elif step == 4:
		print "open RIGHT hand over sensor and ctrl+c to start calibrate min signal"
		step =5;
		np.savetxt(dir_path+"/rlac_calibrate_left_max", angle);
		signal.signal(signal.SIGINT, exit_gracefully)
	elif step == 5:
		print "\nctrl+c to stop calibrate min signal"
		step = 6;
		signal.signal(signal.SIGINT, exit_gracefully)
	elif step == 6:
		print "\nclose RIGHT hand over sensor and ctrl+c to start calibrate max signal"
		step = 7
		np.savetxt(dir_path+"/rlac_calibrate_right_min", angle);
		signal.signal(signal.SIGINT, exit_gracefully)
	elif step == 7:
		print "\nctrl+c to stop calibrate max signal"
		step = 8
		signal.signal(signal.SIGINT, exit_gracefully)
	elif step == 8:
		print "\ncalibration end, please copy the calibration files to the script folder"
		controller.remove_listener(listener)
		np.savetxt(dir_path+"/rlac_calibrate_right_max", angle);
		sys.exit(0)

class SampleListener(Leap.Listener):

	def on_init(self, controller):
		print "Initialized"

	def on_connect(self, controller):
		print "Connected"

	def on_disconnect(self, controller):
		print "Disconnected"

	def on_exit(self, controller):
		print "Exited"

	def on_frame(self, controller):
		global step
		global last_step
		global counter
		global angle
		angle_tmp = np.arange(15,dtype='f').reshape((5,3))

		# Get the most recent frame and report some basic information
		frame = controller.frame()

		if step != last_step:
			counter = 0;
			angle.fill(0)
			last_step = step

		if not (step == 2 or step == 4 or step == 6 or step == 8):
			return
		if (frame.hands.is_empty):
			# print "no hand detected, please put hand over sensor"
			return

		# Get hands
		for hand in frame.hands:
			# step 2 - left min
			# step 4 - left max
			# step 6 - right min
			# step 8 - right max
			if (step == 2 or step == 4) and (not hand.is_left):
				continue;
			if (step == 6 or step == 8) and (hand.is_left):
				continue;
			if hand.confidence < 0.4:
				continue;

			# Get fingers
			for finger in hand.fingers:

				# Get bones
				for b in range(0, 3):
					bone1 = finger.bone(b)
					bone2 = finger.bone(b+1)
					angle[finger.type][b] = (bone1.direction.angle_to(bone2.direction) + angle[finger.type][b]*counter )/(counter+1)
			counter = counter +1
			print angle

if __name__ == '__main__':
	dir_path = os.path.dirname(os.path.realpath(__file__))
	# store the original SIGINT handler
	original_sigint = signal.getsignal(signal.SIGINT)
	signal.signal(signal.SIGINT, exit_gracefully)
	print "put the Leapmotion in front of you on the desk, make the space clear and press ctrl+c"
	# Create a sample listener and controller
	listener = SampleListener()
	controller = Leap.Controller()

	# Have the sample listener receive events from the controller
	controller.add_listener(listener)

	run_program()