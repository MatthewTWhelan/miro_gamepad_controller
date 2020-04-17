#!/usr/bin/python
import rospy
from std_msgs.msg import UInt8, UInt16, UInt32, Float32MultiArray, UInt16MultiArray, UInt32MultiArray
from geometry_msgs.msg import Pose2D, TwistStamped
from sensor_msgs.msg import JointState
import numpy as np
from matplotlib import pyplot as plt
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


	def controller(self):
		t_init = time.time()
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
		LT = joystick.get_axis(2)
		RT = joystick.get_axis(5)
		RS_horizontal = joystick.get_axis(3)
		RS_vertical = joystick.get_axis(4)

		# values take on either 1 if pressed or 0 otherwise
		button_A = joystick.get_button(0)
		button_B = joystick.get_button(1)
		button_X = joystick.get_button(2)
		button_Y = joystick.get_button(3)
		button_LB = joystick.get_button(4)
		button_RB = joystick.get_button(5)
		button_BACK = joystick.get_button(6)
		button_START = joystick.get_button(7)
		button_MENU = joystick.get_button(8)
		button_LS = joystick.get_button(9)
		button_RS = joystick.get_button(10)

		# for directional buttons, hat is a tuple of (left_right, up_down) with values of 1 or -1 if right/up or
		# left/down is pressed, respectively. Both horizontal and vertical can be simultaneously pressed,
		# so that pressing the up and left buttons gives (-1, 1), for instance
		# hat = joystick.get_hat(0)
		#
		#
		vel = - left_stick_vertical * 0.4
		omega = - right_stick_horizontal * 5
		if abs(vel) < 0.05:
			vel = 0
		if abs(omega) < 0.05:
			omega = 0

		if button_A:
			self.victory_dance()
		elif button_B:
			self.head_shake_in_disappointment()


		for i in range(joystick_count):
			joystick = pygame.joystick.Joystick(i)
			joystick.init()



			# print("Joystick {}".format(i))

			# Get the name from the OS for the controller/joystick
			# name = joystick.get_name()
			# print("Joystick name: {}".format(name))

			# Usually axis run in pairs, up/down for one, and left/right for
			# the other.
			axes = joystick.get_numaxes()
			# print("Number of axes: {}".format(axes))

			# for i in range(axes):
			# 	axis = joystick.get_axis(0)
			#
			# 	print("Axis {} value: {:>6.3f}".format(i, axis))
			#
			#
			# buttons = joystick.get_numbuttons()
			#
			# print("Number of buttons: {}".format(buttons))
			#
			#
			# for i in range(buttons):
			# 	button = joystick.get_button(i)
			#
			# 	print("Button {:>2} value: {}".format(i, button))
			#
			#
			# # Hat switch. All or nothing for direction, not like joysticks.
			# # Value comes back in an array.
			# hats = joystick.get_numhats()
			#
			# print("Number of hats: {}".format(hats))
			#
			#
			# for i in range(hats):
			# 	hat = joystick.get_hat(i)
			#
			# 	print("Hat {} value: {}".format(i, str(hat)))


		return vel, omega


	def avoid_wall(self):
		t_init = time.time()
		self.msg_wheels.twist.linear.x = -0.1
		self.msg_wheels.twist.angular.z = 0
		while time.time() - t_init < 1:
			self.pub_wheels.publish(self.msg_wheels)
		p_anti_clock = 0.8
		self.msg_wheels.twist.linear.x = 0
		if np.random.rand() < p_anti_clock:
			self.msg_wheels.twist.angular.z = 1.5
		else:
			self.msg_wheels.twist.angular.z = -1.5
		while time.time() - t_init < 2:
			self.pub_wheels.publish(self.msg_wheels)

	def victory_dance(self):
		t_init = time.time()
		t = 0
		while t < 2:
			self.msg_kin.position[1] = np.radians(t*15 + 30)
			self.pub_kin.publish(self.msg_kin)
			t = time.time() - t_init
		time.sleep(15)

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
		return


if __name__ == "__main__":
	main = MiroController()
	main.controller()
