#!/usr/bin/env python3

import numpy as np
import spatialmath as sm
import spatialgeometry as sg
import tempfile as tf

from roboticstoolbox.robot.ERobot import ERobot
from roboticstoolbox.robot.Link import Link
from roboticstoolbox.robot.ETS import ETS
from roboticstoolbox.robot.ET import ET
from spatialmath import SE3
from roboticstoolbox.tools.data import rtb_path_to_datafile
from distutils.dir_util import copy_tree
from os import mkdir, path


import roboticstoolbox as rtb
import swift

from roboticstoolbox.robot.ERobot import ERobot


# rtb serial robot trial to upload custom URDFs
from math import pi
import os

class Petros(ERobot):
	
	def __init__(self): #, xacro_path):

		link, name, urdf_string, urdf_filepath = self.URDF_read(
		"dual_scara_arm.xacro", 
		tld=os.path.expanduser("~/petricor_sim/src/simple_arm/urdf/")
		)

		# self.base_link = link[0]
		# base_arm = Link(ETS(ET.tz(0.28)), name="base_arm", parent=self.base_link)

		# link[0]._parent = base_arm


		super().__init__(
			link,
			name = "Petros",
			manufacturer = "Custom",
			# gripper_links=[link[4],link[7]],
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

		# isolate right arm links into list
		x=0
		right_arm = []

		for link[x] in link:
			if x>0 and x<5:
				right_arm.append(link[x])
				x=x+1
			else:
				x=x+1

		# print(right_arm)

		self._arm_control() # Will be replaced with a ROS2 timer

	def _arm_control(self):


		## pass #<---- remove this when trying to run this function

		# ------------- This section is reserved for where the arm will be controlled ------------- #

		Tep = sm.SE3.Trans(0.1, 0, 0.1) * sm.SE3.OA([0, 1, 0], [0, 0, -1])
		sol = self.ik_lm_chan(Tep, ilimit=30, tol=1e-05, we=[1, 1, 0, 0, 0, 0])         # solve IK
		print(sol)


		q_pickup = sol[0]
		print(self.fkine(q_pickup))

		# Simple_armTep = (
		# 	sm.SE3.Tx(0.45)
		# 	* sm.SE3.Ty(0.25)
		# 	* sm.SE3.Tz(0.3)
		# 	* sm.SE3.Rx(np.pi)
		# 	* sm.SE3.Rz(np.pi / 2)

			
		# )

		# env = swift.Swift()
		# env.launch(realtime=True)
		# # self.plot(q=self.qr)
		# env.add(self)
		

		# # Target is the maximum reachable position of the arm
		# Simple_arm_Target = sg.Sphere(0.01, color=[0.2, 0.4, 0.65, 0.5], base=Tep)
		# #Frame is the zero position of the arm
		# Simple_arm_frame = sg.Axes(0.1, base=Tep)

		# #Add the target and the frame points to the environment
		# env.add(Simple_arm_Target)
		# env.add(Simple_arm_frame)

		# #Set the initial position of the arm and gripper
		# Simple_arm_frame = sg.Axes(0.1, pose=self.grippers[0].tool)
		# Simple_arm_frame.attach_to(self.grippers[0].links[0])
		# #Add the frame to the environment
		# env.add(Simple_arm_frame)

		# # Construct an ETS for the Simple_arm
		# Simple_arma = self.ets(end=self.grippers[0])
		# arrived = False

		# dt = 0.05

		# gain = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

		# while not arrived:
		# #Returns end effector velocity
		# 	vSimple_arm = rtb.p_servo(Simple_arma.fkine(self.q), Tep, gain=gain, threshold=0.001)
		# #Calculate the new joint angles
		# 	self.qd[Simple_arma.jindices] = np.linalg.pinv(Simple_arma.jacobe(self.q)) @ vSimple_arm

		# env.step(dt) #Step forward the environment

		# env.hold() #Hold the browser window open

		# rtb sr trail - static method

		# def load_my_path():
		# 	#print(__file__)
		# 	os.chdir(os.path.dirname(__file__))

def main(args=None):
	robot = Petros()
	print(robot)

	

	# ROS placholder
	# Node name
	# spin(Node_name)


if __name__ == "__main__":
	main()
