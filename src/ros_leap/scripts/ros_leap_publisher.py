#!/usr/bin/env python

import sys
import signal
import rospy
import Leap
from ros_leap.msg import Leap_Frame, Hand, Finger, Bone, Arm, Vector3

def exit_gracefully(signum, frame):
	# restore the original signal handler as otherwise evil things will happen
	# in raw_input when CTRL+C is pressed, and our signal handler is not re-entrant
	signal.signal(signal.SIGINT, original_sigint)
	controller.remove_listener(listener)
	exit(1)

class LMListener(Leap.Listener):

	def on_init(self, controller):
		print "Initialized"

	def on_connect(self, controller):
		print "Connected"

	def on_disconnect(self, controller):
		print "Disconnected"

	def on_exit(self, controller):
		print "Exited"

	def on_frame(self, controller):
		# Get the most recent frame and report some basic information
		frame = controller.frame()

		if frame.hands.is_empty:
			return
		updated = True
		leap_msg = Leap_Frame()

		leap_msg.header.stamp.secs = frame.timestamp/1000000
		leap_msg.header.stamp.nsecs = frame.timestamp%1000000*1000
		leap_msg.header.frame_id = str(frame.id)
		leap_msg.is_valid = frame.is_valid

		# Get hands
		for hand in frame.hands:

			hand_msg = Hand()
			hand_msg.id = hand.id
			hand_msg.is_left = hand.is_left
			hand_msg.is_right = hand.is_right
			hand_msg.is_valid = hand.is_valid
			hand_msg.confidence = hand.confidence
			hand_msg.direction.x = hand.direction.x
			hand_msg.direction.y = hand.direction.y
			hand_msg.direction.z = hand.direction.z
			hand_msg.palm_normal.x = hand.palm_normal.x
			hand_msg.palm_normal.y = hand.palm_normal.y
			hand_msg.palm_normal.z = hand.palm_normal.z
			hand_msg.palm_position.x = hand.palm_position.x
			hand_msg.palm_position.y = hand.palm_position.y
			hand_msg.palm_position.z = hand.palm_position.z
			hand_msg.palm_velocity.x = hand.palm_velocity.x
			hand_msg.palm_velocity.y = hand.palm_velocity.y
			hand_msg.palm_velocity.z = hand.palm_velocity.z
			hand_msg.palm_width = hand.palm_width
			hand_msg.sphere_center.x = hand.sphere_center.x
			hand_msg.sphere_center.y = hand.sphere_center.y
			hand_msg.sphere_center.z = hand.sphere_center.z
			hand_msg.sphere_radius = hand.sphere_radius
			hand_msg.stabilized_palm_position.x = hand.stabilized_palm_position.x
			hand_msg.stabilized_palm_position.y = hand.stabilized_palm_position.y
			hand_msg.stabilized_palm_position.z = hand.stabilized_palm_position.z
			hand_msg.time_visible = hand.time_visible
			hand_msg.wrist_position.x = hand.wrist_position.x
			hand_msg.wrist_position.y = hand.wrist_position.y
			hand_msg.wrist_position.z = hand.wrist_position.z

			# Get arm bone
			arm = hand.arm
			arm_msg = Arm()
			arm_msg.is_valid = arm.is_valid
			arm_msg.width = arm.width
			arm_msg.direction.x = arm.direction.x
			arm_msg.direction.y = arm.direction.y
			arm_msg.direction.z = arm.direction.z
			arm_msg.elbow_position.x = arm.elbow_position.x
			arm_msg.elbow_position.y = arm.elbow_position.y
			arm_msg.elbow_position.z = arm.elbow_position.z
			arm_msg.wrist_position.x = arm.wrist_position.x
			arm_msg.wrist_position.y = arm.wrist_position.y
			arm_msg.wrist_position.z = arm.wrist_position.z
			hand_msg.arm = arm_msg

			# Get fingers
			for finger in hand.fingers:
				finger_msg = Finger()
				finger_msg.id = finger.id
				finger_msg.is_valid = finger.is_valid
				finger_msg.type = finger.type
				finger_msg.width = finger.width
				finger_msg.length = finger.length
				finger_msg.stabilized_tip_position.x = finger.stabilized_tip_position.x
				finger_msg.stabilized_tip_position.y = finger.stabilized_tip_position.y
				finger_msg.stabilized_tip_position.z = finger.stabilized_tip_position.z
				finger_msg.time_visible = finger.time_visible
				finger_msg.tip_position.x = finger.tip_position.x
				finger_msg.tip_position.y = finger.tip_position.y
				finger_msg.tip_position.z = finger.tip_position.z
				finger_msg.tip_velocity.x = finger.tip_velocity.x
				finger_msg.tip_velocity.y = finger.tip_velocity.y
				finger_msg.tip_velocity.z = finger.tip_velocity.z

				# Get bones
				for b in range(0, 4):
					bone = finger.bone(b)
					bone_msg = Bone()
					bone_msg.type = bone.type
					bone_msg.is_valid = bone.is_valid
					bone_msg.length = bone.length
					bone_msg.width = bone.width
					bone_msg.center.x = bone.center.x
					bone_msg.center.y = bone.center.y
					bone_msg.center.z = bone.center.z
					bone_msg.direction.x = bone.direction.x
					bone_msg.direction.y = bone.direction.y
					bone_msg.direction.z = bone.direction.z
					bone_msg.prev_joint.x = bone.prev_joint.x
					bone_msg.prev_joint.y = bone.prev_joint.y
					bone_msg.prev_joint.z = bone.prev_joint.z
					bone_msg.next_joint.x = bone.next_joint.x
					bone_msg.next_joint.y = bone.next_joint.y
					bone_msg.next_joint.z = bone.next_joint.z
					finger_msg.bones[bone.type] = bone_msg
				hand_msg.fingers[finger.type] = finger_msg
			leap_msg.hands.append(hand_msg)
		rospy.loginfo("Leap Frame %d sent", frame.id)
		pub.publish(leap_msg)

if __name__ == "__main__":
	# store the original SIGINT handler
	original_sigint = signal.getsignal(signal.SIGINT)
	signal.signal(signal.SIGINT, exit_gracefully)
	topic_name = 'leap_frame'
	node_name = 'ros_leap_publisher'
	pub = rospy.Publisher(topic_name, Leap_Frame, queue_size=10)
	rospy.init_node(node_name, anonymous=False)
	# Create a sample listener and controller
	listener = LMListener()
	controller = Leap.Controller()
	# Have the sample listener receive events from the controller
	controller.add_listener(listener)
	rospy.spin()
