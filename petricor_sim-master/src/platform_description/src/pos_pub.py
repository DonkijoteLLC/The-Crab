#!/usr/bin/env python3
# This is the python publisher

import time, rclpy, sys, select, math
from rclpy.node import Node
from rclpy import init, spin
from rclpy.qos import qos_profile_sensor_data
from tf_transformations import euler_from_quaternion

from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

from dataclasses import dataclass

import threading as th

class PosPub(Node):
	
	def __init__(self):
		super().__init__('PosPub')
		
		self.info = "Position controller for robot"
		self.usage = ("w = Forward, A = Left, S = Stop, D = Right, X = Reverse\n"
		"To drive the robot to a desirec location, enter coordinate locations as: X,Y."
		"Enter 'Q' to exit the program.")
		
		# robot position and orientation
		self.robot_x = 0.0
		self.robot_y = 0.0
		self.robot_rot_q = 0.0
		self.robot_rot_theta = 0.0 # rotation about 
		# unit vector describing robot direction
		self.robot_xhat = 0.0
		self.robot_yhat = 0.0

		self.carrot_x = 0.0
		self.carrot_y = 0.0
		self.carrot_xhat = 0.0
		self.carrot_yhat = 0.0
		self.current_angle = 0.0

		self.drivePoints = [] # initialize a blank list of drive points that can be appended later
		
		# Make sure to change the data type for the message 
		# Make sure that the subscriber matches the topic
		self.publish_velocity = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)
		
		# -------- Subscribers --------
		# odometry subscriber (receives the position an orientation of the robot)
		self.subscribe_message = self.create_subscription(Odometry, '/petricor_platform/gazebo_odom', self._sub_callback, qos_profile_sensor_data) # gives 3d pos rot of robot
		
		self.velocity_msg = Float64MultiArray()
		
		period = 0.1 # not sure how fast this pub needs to operate yet
		
		self.timer = self.create_timer(period, self._pub_callback) # timer for the position publisher
		
		# initialize the wheel velocities
		self.wheel_bl = 0.0	# back left
		self.wheel_br = 0.0	# back right
		self.wheel_fl = 0.0	# front left
		self.wheel_fr = 0.0	# front right
		
		# create a thread for the publisher
		self.get_logger().info('Starting thread')
		spin_thread = th.Thread(target=rclpy.spin, args=(self, ), daemon=True)
		spin_thread.start()
		
		
	def _sub_callback(self, msg):
		# robot position is in the msg
		self.robot_x = msg.pose.pose.position.x # robot x position
		self.robot_y = msg.pose.pose.position.y # robot y position
		self.robot_rot_q = msg.pose.pose.orientation # rotation about quaternion axis
		(roll, pitch, theta) = euler_from_quaternion([self.robot_rot_q.x, self.robot_rot_q.y, self.robot_rot_q.z, self.robot_rot_q.w])
		self.robot_rot_theta = theta # get the robot's rotation (about Z)
		self._UpdateRobotVectorDirection()
		
	def _pub_callback(self):
		self.velocity_msg.data = [self.wheel_bl, self.wheel_br, self.wheel_fl, self.wheel_fr]
		# self.get_logger().info('PosPub is publishing!', throttle_duration_sec = 2) # based on position, can change message
		self.publish_velocity.publish(self.velocity_msg)
			
		
	def _straight_line(self, wheelSpeed):
		# all wheels move at the same wheel speed
		self.wheel_bl = wheelSpeed
		self.wheel_br = wheelSpeed
		self.wheel_fl = wheelSpeed
		self.wheel_fr = wheelSpeed
		
	def _stop(self):
		# stop all wheels
		self.wheel_bl = 0.0
		self.wheel_br = 0.0
		self.wheel_fl = 0.0
		self.wheel_fr = 0.0
		
	def _skid_right(self, wheelSpeed):
		# skid steer right
		self.wheel_bl = wheelSpeed
		self.wheel_br = -wheelSpeed
		self.wheel_fl = wheelSpeed
		self.wheel_fr = -wheelSpeed
		
	def _skid_left(self, wheelSpeed):
		# skid steer left
		self.wheel_bl = -wheelSpeed
		self.wheel_br = wheelSpeed
		self.wheel_fl = -wheelSpeed
		self.wheel_fr = wheelSpeed
		
	def _turn(self, leftSpeed, rightSpeed):
		# most flexible out of all of the drive functions
		# simply set the desired wheel velocities for the left and right wheels
		self.wheel_bl = leftSpeed
		self.wheel_br = rightSpeed
		self.wheel_fl = leftSpeed
		self.wheel_fr = rightSpeed

	def _GetRotation(self, X1, Y1, X2, Y2):
		# atan2 returns an angle from -pi to pi
		# the start location is X1,Y1 and goes to X2,Y2
		angle = 0.0
		angle = math.atan2(Y2 - Y1, X2 - X1)
		# angle = math.atan2(Y2 - Y1, X2 - X1) * 180 / math.pi # get angle in degrees (debug)
		return angle

	def _UpdateRobotVectorDirection(self):
		# Update the robot's unit vector direction based on the heading
		self.robot_xhat = math.cos(self.robot_rot_theta)
		self.robot_yhat = math.sin(self.robot_rot_theta)

	def _CarrotUnitVector(self):
		# calculates the components of the unit vector of the direction from the robot to the carrot
		# magnitude is the distance from the robot to the carrot
		magnitude = math.sqrt(math.pow(self.carrot_x-self.robot_x,2) + math.pow(self.carrot_y-self.robot_y,2))
		# carrot_xhat is the x component of the unit vecot
		self.carrot_xhat = (self.carrot_x - self.robot_x) / magnitude
		# carrot_yhat is the y component of the unit vector
		self.carrot_yhat = (self.carrot_y - self.robot_y) / magnitude
	
	def _DriveAngle(self):
		# Calculates the angle that the robot needs to turn to align with the carrot or desired destination
		# atan2(r_hat × c_hat, r_hat ∙ c_hat)
		# r_hat is the unit vector of the robot's current direction
		# c_hat is the unit vector of the direction from the robot to the carrot
		cross = self.robot_xhat * self.carrot_yhat - self.robot_yhat * self.carrot_xhat
		dot = self.robot_xhat * self.carrot_xhat + self.robot_yhat * self.carrot_yhat
		angle = math.atan2(cross, dot)
		return angle

	def _ConvertToDegrees(self, angle):
		# Helpful when debugging
		degAngle = angle * 180 / math.pi
		return degAngle

	def _driveToCarrot(self):
		atEndLocation = False
		#rotate to get aligned
		self._stop() # stop the robot if the robot was commanded to move before
		self._CarrotUnitVector()
		angle = self._DriveAngle()
		print("drive angle1 = " + str(angle))
		tolerance = 0.02 # about 1.15 degrees
		aligned = False # set flag stating that the robot is not aligned
		print("drive angle2 = " + str(angle))
		print("robot orientation " + str(self.robot_rot_theta))
		# don't think wrapping the angles are needed
		if abs(angle/2) < tolerance:  #self.robot_rot_theta >= angle  - tolerance and self.robot_rot_theta <= angle + tolerance:
			# robot does not need to rotate because it is within the tolerance
			self._stop() # just a precaution
			aligned = True # set aligned flag
			print("robot orientation " + str(self.robot_rot_theta))
		else:
			# move the robot until the robot is within the tolerance window
			while not aligned:
				#self._UpdateRobotVectorDirection()
				self._CarrotUnitVector()
				angle = self._DriveAngle()
				if angle < 0: 
					# robot angle is 
					self._skid_right(2.0)
				else:
					self._skid_left(2.0)
				time.sleep(0.1) # wait and check if robot aligned
				if abs(angle/2) < tolerance:
					# the robot is aligned
					self._stop() # stop rotating the robot
					aligned = True # set the flag to exit the loop and continue to next step

		# drive forward while adjusting rotation
		posTolerance = 0.1 # can adjust
		while not atEndLocation:
			# this is a simple drive model that does not ramp up the drive speed if far way (i.e., static velocities)
			#self._UpdateRobotVectorDirection()
			self._CarrotUnitVector()
			angle = self._DriveAngle()
			if abs(angle/2) < tolerance:
				self._straight_line(3.0)
			elif angle < 0:
				self._turn(4.0, 2.0)
			else:
				self._turn(2.0, 4.0)
			time.sleep(0.1) # wait and check if robot aligned
			if abs(self.robot_x - self.carrot_x) <= posTolerance and abs(self.robot_y - self.carrot_y) <= posTolerance:
				atEndLocation = True
				self._stop()
				print("At end location")
		
	def run(self):
		# This is a infinite while loop that can be exited via user input.
		# The publisher and subscriber run in a separate thread while this method
		# takes the user's input to move the robot or perform other functions.
		aborted = False # flag to exit user interface loop and code
		print(self.info)
		print(self.usage)
		self.loop_rate = self.create_rate(100, self.get_clock())
		while rclpy.ok() and not aborted:
			# Check for user input to adjust the robot position or abort
			try:
				user_input = ""
				# Accepted input (X first followed by Y)
				#	x,y

				# get user input
				user_input = sys.stdin.readline().strip() # take out whitespace
				
				user_input = user_input.upper() # convert to uppercase
				# if the user wants help or some other comm
				if "HELP" in user_input:
					print(self.usage)
					user_input = "" # reset input and get new input
					continue # move on
				if "Q" in user_input:
					aborted = True
					print("Quitting")
					continue # move on

				# send directional input (good for testing purposes)
				if "W" in user_input:
					self._straight_line(2.0)
					print("Moving Forward")
					continue # move on
				if "A" in user_input:
					self._skid_left(2.0)
					print("Turning Left")
					continue # move on
				if "D" in user_input:
					self._skid_right(2.0)
					print("Turning Right")
					continue # move on
				if "X" in user_input:
					self._straight_line(-2.0)
					print("Moving in Reverse")
					continue # move on
				if "S" in user_input:
					self._stop()
					print("Robot stopped")
					continue # move on
				if "T" in user_input:
					self._turn(0.0, 4.0)
					print("Turning")
					continue # move on
				if "O" in user_input:
					with open('gPointData.txt') as f:
						lines = f.readlines()
						for line in lines:
							data = line.split() # split based on whitespace
							#TODO Parse the X and Y components out and save into point list
					continue # move on
				# could add steering combinations (e.g., W & A would cause left and forward)

				for subList in user_input.split(";"):
					pointData = subList.split(",")
					if len(pointData) > 1:
						if pointData[0].isnumeric and pointData[1].isnumeric:
							p = Point(float(pointData[0]),float(pointData[1]))
							self.drivePoints.append(p)
					else:
						# the entered valued were non-numeric input
						print("Please enter numeric coordinates/enter the coordinate location using the following format: X,Y")
						continue # eventually need to send request for new points since something was entered wrong or corrupted
				for p in self.drivePoints:
					self.carrot_x = p.x
					self.carrot_y = p.y
					# call drive function (in future, somehow add mechanism to avoid obstacles?)
					self._driveToCarrot()
				# clear the drive points after iterrating throught the list
				self.drivePoints.clear()
			except Exception as e:
				self.drivePoints.clear() # clear the drive points (don't want old data)
				continue # TODO fix this based on the error
				#log that an issue occurred

@dataclass
class Point:
	x: float
	y: float
	z: float = 0.0

@dataclass
class GCodeData:
	X: float # 
	Y: float
	Z: float
	I: float
	J: float
	

def main(args=None):
	init(args=args)
	PosPubNode = PosPub() # if multiple classes, don't create multiple nodes
	PosPubNode.run()
	#spin(PosPubNode) # this is a blocking call (can't do user input) (i.e., infinite while loop)
	
	
if __name__ == '__main__': # <-- This is where the code "starts"
	main() # <-- This executes the 'main()' function of the code
