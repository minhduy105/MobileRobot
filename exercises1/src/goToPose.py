#!/usr/bin/env python

import rospy
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from std_msgs.msg import String
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

class Navigation():

	def __init__(self):
		self.positionA = {'x': 0.2676058, 'y' : 0.52807505}
		self.positionB = {'x': 1.4213784, 'y' : 1.25394585}
		self.positionC = {'x': 2.3626056, 'y' : 0.50115252}
		self.positionD = {'x': 0.9985957, 'y' : -0.34325492}


	def startNav(self):
		rospy.init_node('patrol')
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.client.wait_for_server()
		
		goal = self.goal_pose(self.positionA)
		print (goal)
		self.client.send_goal(goal)
		success = self.client.wait_for_result(rospy.Duration(60)) # let it try to get there in 1 minute 
		state = self.client.get_state()
		if success and state == GoalStatus.SUCCEEDED:
			print "get to A"
		else:
			self.client.cancel_goal()
		
		goal = self.goal_pose(self.positionB)
		self.client.send_goal(goal)
		success = self.client.wait_for_result(rospy.Duration(60)) # let it try to get there in 1 minute 
		state = self.client.get_state()
		if success and state == GoalStatus.SUCCEEDED:
			print "get to B"
		else:
			self.client.cancel_goal()

		goal = self.goal_pose(self.positionC)
		self.client.send_goal(goal)
		success = self.client.wait_for_result(rospy.Duration(60)) # let it try to get there in 1 minute 
		state = self.client.get_state()
		if success and state == GoalStatus.SUCCEEDED:
			print "get to C"
		else:
			self.client.cancel_goal()
		
		goal = self.goal_pose(self.positionD)
		self.client.send_goal(goal)
		success = self.client.wait_for_result(rospy.Duration(60)) # let it try to get there in 1 minute 
		state = self.client.get_state()
		if success and state == GoalStatus.SUCCEEDED:
			print "get to D"
		else:
			self.client.cancel_goal()
	

	def goal_pose(self, pose):
		goal_pose = MoveBaseGoal()
		goal_pose.target_pose.header.frame_id="map"
		goal_pose.target_pose.header.stamp = rospy.Time.now()
		goal_pose.target_pose.pose = Pose(Point(pose['x'], pose['y'], 0.000),
									Quaternion(0.000, 0.000, 0.000, 1.000))
		return goal_pose


if __name__ == '__main__':
	x = Navigation()
	x.startNav()
	rospy.spin()


