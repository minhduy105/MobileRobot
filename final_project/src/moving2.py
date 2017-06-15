#!/usr/bin/env python

from __future__ import print_function
import csv
import rospkg
import rospy
import actionlib
import time
import copy


from sensor_msgs.msg import Image
from std_msgs.msg import String
from PIL import Image as Img
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class Scheduler():
	def __init__(self):
		self.in_motion = False
		self.continous_mode = False
		self.home_requested = 0
		self.get_to_location = True
		self.get_name = False
		# Mtion related variables
		self.setUpPointways()
		self.setUpSpinning()

		self.move_base = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		self.move_base.wait_for_server(rospy.Duration(5))
		self.goal = MoveBaseGoal()

		key_sub = rospy.Subscriber('keys', String, self.keyCallback)

	def setUpPointways(self):
		pointA = [0.473, 1.870, 0.0, 0.0, 0.0, -0.653, 0.758]
		pointB = [0.585, -0.885, 0.0, 0.0, 0.0, -0.653, 0.758]
		self.waypoints = []
		self.waypoints.append(pointA)
		self.waypoints.append(pointB)
		# self.waypoints.append(pointA)
		self.waypoints_idx = 0

	def setUpSpinning(self):
		spin1 = [0.0, 0.0, 0.0, 0.0, 0.0, -0.653, 0.758]
		spin2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.869, -0.496]
		spin3 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.998, -0.061]
		spin4 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.923, 0.385]
		spin5 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.712, 0.702]
		spin6 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.402, 0.916]
		spin7 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.057, 0.998]
		spin8 = [0.0, 0.0, 0.0, 0.0, 0.0, -0.344, 0.939]
		self.spinning = []
		self.spinning.append(spin1)
		self.spinning.append(spin2)
		self.spinning.append(spin3)
		self.spinning.append(spin4)
		self.spinning.append(spin5)
		self.spinning.append(spin6)
		self.spinning.append(spin7)
		self.spinning.append(spin8)
		self.spinning.append(spin1)
		self.spin_idx = 0



	def keyCallback(self, keys):
		#have get_name there so it do not do anythings at home 
		if keys.data == "s":
			print ("start")
			self.waypoints_idx = 0
			self.goTo()

	def goTo(self):
		if self.waypoints_idx < len(self.waypoints):
			if not self.in_motion and self.get_to_location:
				self.goal.target_pose.header.frame_id= 'map'
				self.goal.target_pose.header.stamp = rospy.Time.now()

				self.goal.target_pose.pose.position.x = float(self.waypoints[self.waypoints_idx][0])
				self.goal.target_pose.pose.position.y = float(self.waypoints[self.waypoints_idx][1])
				self.goal.target_pose.pose.position.z = float(0.0)
				
				self.goal.target_pose.pose.orientation.x = float(0.0)
				self.goal.target_pose.pose.orientation.y = float(0.0)
				self.goal.target_pose.pose.orientation.z = float(self.waypoints[self.waypoints_idx][5])
				self.goal.target_pose.pose.orientation.w = float(self.waypoints[self.waypoints_idx][6])
				self.get_to_location = False
				self.in_motion = True        
				
				print("Traveling to waypoint with pose")
				print(self.goal)
				self.move_base.send_goal(self.goal, self.goToComplete)
				print("Goal sent")
	
	def goToComplete(self,a,b):
		print("Goal Completed")
		self.in_motion = False
		self.get_to_location = True
		self.spin_idx = 0
		self.spin()

	def spin(self):
		if self.spin_idx < len(self.spinning):
			if not self.in_motion and self.get_to_location:
				self.goal.target_pose.header.frame_id= 'map'
				self.goal.target_pose.header.stamp = rospy.Time.now()

				self.goal.target_pose.pose.position.x = float(self.waypoints[self.waypoints_idx][0]) 
				self.goal.target_pose.pose.position.y = float(self.waypoints[self.waypoints_idx][1])
				self.goal.target_pose.pose.position.z = float(0.0)
				
				self.goal.target_pose.pose.orientation.x = float(0.0)
				self.goal.target_pose.pose.orientation.y = float(0.0)
				self.goal.target_pose.pose.orientation.z = float(self.spinning[self.spin_idx][5])
				self.goal.target_pose.pose.orientation.w = float(self.spinning[self.spin_idx][6])
				self.get_to_location = False
				self.in_motion = True        
				
				print("Star spinning")
				print(self.goal)
				self.move_base.send_goal(self.goal, self.spinComplete)
				print("Goal sent")	


	def spinComplete(self,c,d):
		print("Goal Completed")
		self.in_motion = False
		self.get_to_location = True
		if self.spin_idx < len(self.spinning):
			self.spin()		
			self.spin_idx += 1
			print (self.spin_idx)
			print (len(self.spinning))	
		else:
			print ("Go away")
			self.waypoints_idx += 1
			self.goTo()

if __name__ == '__main__':
	rospy.init_node('moving')
	
	scheduler = Scheduler()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
