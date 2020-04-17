#!/usr/bin/python
import rospy
from std_msgs.msg import UInt8, UInt16, UInt32, Float32MultiArray, UInt16MultiArray, UInt32MultiArray
from geometry_msgs.msg import Pose2D, TwistStamped
from sensor_msgs.msg import JointState
import numpy as np
import os
import time
import miro2 as miro
import pygame

class MiroController:

	def __init__(self):
		print("Running Controller...")

		rospy.init_node("miro_controller")

		# robot name
		topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

		# publish to wheels
		topic = topic_base_name + "/control/cmd_vel"
		self.pub_wheels = rospy.Publisher(topic, TwistStamped, queue_size=0)
		self.msg_wheels = TwistStamped()

		# publish to kinematic joints
		topic = topic_base_name + "/control/kinematic_joints"
		self.pub_kin = rospy.Publisher(topic, JointState, queue_size=0)
		self.msg_kin = JointState()
		self.msg_kin.position = [0.0, np.radians(30.0), 0.0, 0.0]

		# subscribe to kinematic joints
		topic = topic_base_name + "/sensors/kinematic_joints"
		self.sub_kin = rospy.Subscriber(topic, JointState, self.callback_kinematic_joints, queue_size=5,
		                                tcp_nodelay=True)
		self.kin_pos = [0.0, np.radians(30.0), 0.0, 0.0] #[tilt, lift, yaw, pitch]

		# publish to cosmetic joints
		topic = topic_base_name + "/control/cosmetic_joints"
		self.pub_cos = rospy.Publisher(topic, Float32MultiArray, queue_size=0)
		self.msg_cos = Float32MultiArray()
		self.msg_cos.data = [0.5, 0.5, 0.0, 0.0, 0.5, 0.5] # all between 0 and 1 [tail droop, tail wag, left eyelid,
		# right eyelid, left ear, right ear]

		# setup controller
		pygame.init()
		pygame.joystick.init()
		self.joystick1 = pygame.joystick.Joystick(0)
		self.joystick1.init()

	def callback_kinematic_joints(self, msg):
		self.kin_pos[0] = msg.position[0]
		self.kin_pos[1] = msg.position[1]
		self.kin_pos[2] = msg.position[2]
		self.kin_pos[3] = msg.position[3]

	def controller(self):
		t_init = time.time()
		self.reset_miro()
		vel = 0
		omega = 0
		while not rospy.core.is_shutdown():
			t = time.time() - t_init

			vel, alpha = self.get_movement_from_joystick_vals()
			self.msg_wheels.twist.linear.x = vel
			self.msg_wheels.twist.angular.z = alpha

			self.pub_wheels.publish(self.msg_wheels)

	def get_movement_from_joystick_vals(self):
		# max and min velocities are +- 0.4 m/s
		# max and min angular velocities are +- 5 rad/s
		vel = 0
		omega = 0

		pygame.event.get() # gets any updates from pygame events (events such as a button been pressed etc)
		# for a in range(self.joystick1.get_numaxes()):
		# 	print("Axis number %d has value %d \n" %(a, self.joystick1.get_axis(a)))
		joystick_count = pygame.joystick.get_count()

		joystick = pygame.joystick.Joystick(0) # use the first joystick recognised

		# values range from -1 to 1 for the axis
		LS_horizontal = joystick.get_axis(0)
		LS_vertical = joystick.get_axis(1)
		RS_horizontal = joystick.get_axis(3)
		RS_vertical = joystick.get_axis(4)
		# LT = joystick.get_axis(2)
		# RT = joystick.get_axis(5)
		#
		# # values take on either 1 if pressed or 0 otherwise
		# button_A = joystick.get_button(0)
		# button_B = joystick.get_button(1)
		# button_X = joystick.get_button(2)
		# button_Y = joystick.get_button(3)
		# button_LB = joystick.get_button(4)
		# button_RB = joystick.get_button(5)
		# button_BACK = joystick.get_button(6)
		# button_START = joystick.get_button(7)
		# button_MENU = joystick.get_button(8)
		# button_LS = joystick.get_button(9)
		# button_RS = joystick.get_button(10)

		# for directional buttons, hat is a tuple of (left_right, up_down) with values of 1 or -1 if right/up or
		# left/down is pressed, respectively. Both horizontal and vertical can be simultaneously pressed,
		# so that pressing the up and left buttons gives (-1, 1), for instance
		# hat = joystick.get_hat(0)
		#
		#
		vel = - LS_vertical * 0.4
		omega = - RS_horizontal * 5

		# To avoid small amounts of drift
		if abs(vel) < 0.05:
			vel = 0
		if abs(omega) < 0.05:
			omega = 0

		# Mapping the buttons to sequences of behaviours
		# if button_A == 1:
		# 	self.victory_dance()
		# 	button_A = 0
		# elif button_B == 1:
		# 	self.head_shake_in_disappointment()
		# elif button_X == 1:
		# 	self.reset_miro()

		return vel, omega


	def victory_dance(self):
		t_init = time.time()
		t = 0
		while t < 2:
			self.msg_kin.position[1] = np.radians(t*15 + 30)
			self.pub_kin.publish(self.msg_kin)
			t = time.time() - t_init
		time.sleep(10)
		self.reset_miro()

	def head_shake_in_disappointment(self):
		t_init = time.time()
		self.msg_wheels.twist.linear.x = 0
		self.msg_wheels.twist.angular.z = 0
		t = 0
		while t < 6:
			t = time.time() - t_init
			if t < 2:
				self.msg_kin.position[1] = np.radians(t*15 + 34) # lift, between 8 degs (all way up) and 60 deg (all way
		# down)
				self.msg_cos.data[2] = t - 0.2
				self.msg_cos.data[3] = t - 0.2
			else:
				self.msg_kin.position[2] = np.radians(30.0 * np.sin((t - 1) / (8/3) * 2 * np.pi)) # yaw, between -50 and
			# 50 degs
			self.msg_kin.position[3] = np.radians(0.0) #  # pitch, between -22 degs (up) and 8 degs (down)
			self.pub_kin.publish(self.msg_kin)
			self.pub_cos.publish(self.msg_cos)
		self.reset_miro()

	def reset_miro(self):
		# [tilt, lift, yaw, pitch] range of vals (in degs) are [unknown, 8 (up) to 60 (down), -50 (right) to 50 (
		# left), -22 (up) to 8 (down)]

		reset_position = [0.0, np.radians(30), np.radians(0), 0.0]
		current_position = self.kin_pos
		[diff_tilt, diff_lift, diff_yaw, diff_pitch] = [a - b for a, b in zip(current_position, reset_position)]
		while abs(diff_lift) > 0.1 or abs(diff_pitch) > 0.05 or abs(diff_yaw) > 0.05:
		# while abs(diff_lift) > 0.1:
		# 	print("Updating")
			print([diff_tilt, diff_lift, diff_yaw, diff_pitch])
			current_position = self.kin_pos
			[diff_tilt, diff_lift, diff_yaw, diff_pitch] = [a - b for a, b in zip(current_position, reset_position)]
			self.msg_kin.position[0] += - 0.0005 * diff_tilt
			self.msg_kin.position[1] += - 0.001 * diff_lift
			self.msg_kin.position[2] += - 0.001 * diff_yaw
			self.msg_kin.position[3] += - 0.001 * diff_pitch
			self.pub_kin.publish(self.msg_kin)


if __name__ == "__main__":
	main = MiroController()
	main.controller()
