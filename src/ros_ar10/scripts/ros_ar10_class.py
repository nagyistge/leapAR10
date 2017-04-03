#!/usr/bin/env python

"""
Active8 Robots, AR10 hand class.

Includes pololu serial communication, speed, acceleration
and movement together with demonstration facilities.
"""

import time
import sys
import random
import serial
import csv

class ar10:
	def __init__(self,device_path, calibration_file_path):
		self.speed        = 20
		self.acceleration = 10

		self.intercept = []
		self.slope     = []

		# When connected via USB, the Maestro creates two virtual serial ports
		# /dev/ttyACM0 for commands and /dev/ttyACM1 for communications.
		# Be sure the Maestro is configured for "USB Dual Port" serial mode.
		# "USB Chained Mode" may work as well, but hasn't been tested.

		# Pololu protocol allows for multiple Maestros to be connected. A device
		# number is used to index each connected unit.  This code currently is statically
		# configured to work with the default device 0x0C (or 12 in decimal).

		# Open the command port
		self.usb = serial.Serial(device_path, baudrate=9600)
		# Command lead-in and device 12 are sent for each Pololu serial commands.
		self.pololu_command = chr(0xaa) + chr(0xc)

		#Read the calibration file
		try:
			cal_file = csv.reader(open(calibration_file_path), delimiter='\t')
			for row in cal_file:
				self.intercept.append(float(row[1]))
				self.slope.append(float(row[2]))
		except IOError:
			print "Calibration file missing"
			print "Please run ros_ar10_calibrate.py"

	# Cleanup by closing USB serial port
	def close(self):
		self.usb.close()

	# Change speed setting
	def change_speed(self, speed):
		self.speed = speed

	# Set speed of channel
	def set_speed(self, channel):
		lsb = self.speed & 0x7f                      #7 bits for least significant byte
		msb = (self.speed >> 7) & 0x7f               #shift 7 and take next 7 bits for msb
		# Send Pololu intro, device number, command, channel, speed lsb, speed msb
		command = self.pololu_command + chr(0x07) + chr(channel) + chr(lsb) + chr(msb)
		self.usb.write(command)

	# Change acceleration setting
	def change_acceleration(self, acceleration):
		self.acceleration = acceleration

	# Set acceleration of channel
	# This provide soft starts and finishes when servo moves to target position.
	def set_acceleration(self, channel, acceleration):
		lsb = acceleration & 0x7f                      # 7 bits for least significant byte
		msb = (acceleration >> 7) & 0x7f               # shift 7 and take next 7 bits for msb
		# Send Pololu intro, device number, command, channel, acceleration lsb, acceleration msb
		command = self.pololu_command + chr(0x09) + chr(channel) + chr(lsb) + chr(msb)
		self.usb.write(command)

	# Set channel to a specified target value
	def set_target(self, channel, target):
		lsb = target & 0x7f                      # 7 bits for least significant byte
		msb = (target >> 7) & 0x7f               # shift 7 and take next 7 bits for msb
		# Send Pololu intro, device number, command, channel, and target lsb/msb
		command = self.pololu_command + chr(0x04) + chr(channel) + chr(lsb) + chr(msb)
		self.usb.write(command)

	# convert joint number to channel number
	def joint_to_channel(self, joint):
		channel = joint + 10
		return channel

	# Get the current position of the device on the specified channel
	# The result is returned in a measure of quarter-microseconds, which mirrors
	# the Target parameter of set_target.
	# This is not reading the true servo position, but the last target position sent
	# to the servo.  If the Speed is set to below the top speed of the servo, then
	# the position result will align well with the acutal servo position, assuming
	# it is not stalled or slowed.
	def get_set_position(self, joint):
		# convert joint to channel
		channel = self.joint_to_channel(joint)

		command = self.pololu_command + chr(0x10) + chr(channel)
		self.usb.write(command)
		lsb = ord(self.usb.read())
		msb = ord(self.usb.read())

		return (msb << 8) + lsb

	# Have servo outputs reached their targets? This is useful only if Speed and/or
	# Acceleration have been set on one or more of the channels.  Returns True or False.
	def get_read_position(self, channel):
		command = self.pololu_command + chr(0x90) + chr(channel)
		self.usb.write(command)
		lsb = ord(self.usb.read())
		msb = ord(self.usb.read())
		read_position = (256 * msb) + lsb

		return read_position

	# Have servo outputs reached their targets? This is useful only if Speed and/or
	# Acceleration have been set on one or more of the channels.  Returns True or False.
	def get_position(self, channel):
		read_position = self.get_read_position(channel)
		position      = self.intercept[channel] + (self.slope[channel] * read_position)

		return position

	# Have servo outputs reached their targets? This is useful only if Speed and/or
	# Acceleration have been set on one or more of the channels.  Returns True or False.
	def get_moving_state(self):
		command = self.pololu_command + chr(0x13) + chr(0x01)
		self.usb.write(command)
		if self.usb.read() == chr(0):
			return False
		else:
			return True

	# Run a Maestro Script subroutine in the currently active script.  Scripts can
	# have multiple subroutines, which get numbered sequentially from 0 on up.  Code your
	# Maestro subroutine to either infinitely loop, or just end (return is not valid).
	def run_script(self, subNumber):
		command = self.pololu_command + chr(0x27) + chr(subNumber)
		# can pass a param with comman 0x28
		#  command = self.pololu_command + chr(0x28) + chr(subNumber) + chr(lsb) + chr(msb)
		self.usb.write(command)

	# Stop the current Maestro Script
	def stop_script(self):
		command = self.pololu_command + chr(0x24)
		self.usb.write(command)

	# move joint to target position
	def move(self, joint, target):
		# convert joint to channel
		channel = self.joint_to_channel(joint)

		# check target position is in range
		if target > 7950:
			target = 7950
		elif target < 4200:
			target = 4200

		# a speed of 1 will take 1 minute
		# a speed of 60 would take 1 second.
		# Speed of 0 is unlimited
		self.set_speed(channel)

		# Valid values are from 0 to 255. 0=unlimited, 1 is slowest start. 
		# A value of 1 will take the servo about 3s to move between 1ms to 2ms range.
		self.set_acceleration(channel, self.acceleration)

		# Valid servo range is 256 to 3904
		self.set_target(channel, target)

	# wait for joints to stop moving
	def wait_for_hand(self):
		while self.get_moving_state():
			time.sleep(0.25)

	def open_hand(self):
		print ('Openning ...')
		self.move(8,7900) # thumb 0
		self.move(9,7900) # thumb 1
		self.move(0,7900) # index 0
		self.move(1,7900) # index 1
		self.move(2,7900) # middle 0
		self.move(3,7900) # middle 1
		self.move(4,7900) # ring 0
		self.move(5,7900) # ring 1
		self.move(6,7900) # pinky 0
		self.move(7,7900) # pinky 1
		self.wait_for_hand()
		print ('Open complete')

	def close_hand(self):
		print ('Closing ...')
		self.move(0,4500) # index 0
		self.move(1,4500) # index 1
		self.move(2,4500) # middle 0
		self.move(3,4500) # middle 1
		self.move(4,4500) # ring 0
		self.move(5,4500) # ring 1
		self.move(6,4500) # pinky 0
		self.move(7,4500) # pinky 1
		self.wait_for_hand()
		self.move(8,6000) # thumb 0
		self.move(9,7000) # thumb 1
		self.wait_for_hand()
		print ('close complete')