#!/usr/bin/python3

import time, rclpy, sys, select, math
import numpy as np
import spatialmath as sm
import spatialgeometry as sg
import tempfile as tf

from rclpy.node import Node
from rclpy import init, spin
from roboticstoolbox.robot.ERobot import ERobot
from roboticstoolbox.robot.Link import Link
from roboticstoolbox.robot.ETS import ETS
from roboticstoolbox.robot.ET import ET
from spatialmath import SE3
from roboticstoolbox.tools.data import rtb_path_to_datafile
from distutils.dir_util import copy_tree
from os import mkdir, path
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from sensor_msgs.msg import JointState

import roboticstoolbox as rtb


from roboticstoolbox.robot.ERobot import ERobot

from math import pi
import os
import threading as th

class RobotDefine(ERobot):
	def __init__(self):

		link, name, urdf_string, urdf_filepath = self.URDF_read(
		"dual_scara_arm.xacro", 
		tld=os.path.expanduser("~/petricor_sim/src/simple_arm/urdf/")
		)		

		super().__init__(
			link,
			name = "Petros",
			manufacturer = "Custom",
			urdf_string=urdf_string,
			urdf_filepath=urdf_filepath
			)
		

		self.qdlim = np.array(
			[
				4.0,
				4.0,
				4.0,
				4.0,
				2.0,
				2.0,
				2.0,
				2.0
			]
		)
		# Place holder values for qdlim
		# 4.0 for velocity & 2.0 for acceleration?

		# self.qr = np.array([60, 45, -60, -45, 60])
		# # trail values for the "Ready" Postion
		# self.qz = np.zeros(5)

		# self.addconfiguration("qr", self.qr)
		# self.addconfiguration("qz", self.qz)

	def _get_angles(self, x, y, z):
		self.Tep = sm.SE3.Trans(x, y, z) * sm.SE3.OA([0, 1, 0], [0, 0, -1]) # desired xy pos
		self.sol = self.ik_lm_chan(self.Tep, ilimit=30, tol=1e-05, we=[1, 1, 0, 0, 0, 0])  # solve IK

		return self.sol

class Petros(Node):
	static_member_var = 10

	def __init__(self):
		super().__init__('Petros')

		# ---------- Publishers -----------
		self.joint_position_control_publisher = self.create_publisher(Float64MultiArray, '/position_controllers/commands', 10)


		# goal_pose setpoint publisher
		pub_period = 1.0/1000.0
		self.cmd_pub = self.create_timer(pub_period, self.cmd_pub_cb)

		# Spin in a separate thread
		self.spin_thread = th.Thread(target=rclpy.spin, args=(self, ), daemon=True)
		self.spin_thread.start()


		# ---------- Subscribers ----------
		# self.joint_state_subscriber = self.create_subscription(JointState, 'sensor_msgs/msg/JointState', 10)

		# ---------- Initialization --------
		self.joint_angle = Float64MultiArray()
		self.joint_angle.data = [0.0, 0.25] # Initialize with valid elbow, shoulder positions
		self.simple_arm = RobotDefine()

	def _pos_query(self):

		self.loop_rate = self.create_rate(10, self.get_clock())
		self.get_logger().info("X, Y, Z of EE")

		while rclpy.ok():
			user_input = input("Enter comma deliminated float: ")
			stripped = [float(val) for val in user_input.split(',')]

			x = stripped[0]
			y = stripped[1]
			z = stripped[2]

			sol = self.simple_arm._get_angles(x, y, z)
			self.angle_calc = sol[0]
			self._actuate()

	def _actuate(self):

		# self.joint_angle.data = [self.angle_calc[1], self.angle_calc[0], self.angle_calc[2]]
		# Directly send elbow, shoulder (ros2_control does alphabetical order)
		self.joint_angle.data = [self.angle_calc[2], self.angle_calc[1]]

		#self.joint_position_control_publisher.publish(self.joint_angle)	
	def cmd_pub_cb(self):
		# Elbow: 0.1 -> 1.75
		# Shoulder: -1 -> 1
		# Z: 0.01 -> 0.27
		self.joint_angle.layout.dim = [MultiArrayDimension()]
		self.joint_angle.layout.dim[0].size = 2 # only sending revolute commands for now
		self.joint_angle.layout.dim[0].stride = 1
		self.joint_angle.layout.dim[0].label = "joint_cmds"
		self.joint_position_control_publisher.publish(self.joint_angle)	

		
def main(args=None):
	init(args=args)
	Petros_node = Petros()
	Petros_node._pos_query()

if __name__ == "__main__":
	main()