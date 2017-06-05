#!/usr/bin/env python

import rospy
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from std_msgs.msg import String
from actionlib_msgs.msg import *

class Navigation():

	def __init__(self):
		self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.move_base.wait_for_server(rospy.Duration(5))
		
	def startNav(self,goal_pos):

		success = False
		goal = self.goal_pose(goal_pos)
		self.move_base.send_goal(goal)
			

		success = self.move_base.wait_for_result(rospy.Duration(60)) # let it try to get there in 1 minute 
		state = self.move_base.get_state()

		if not success: 
			print "hell no"
			self.move_base.cancel_goal()
	

	def goal_pose(self, pose):
		goal_pose = MoveBaseGoal()
		goal_pose.target_pose.header.frame_id = 'map'
		goal_pose.target_pose.header.stamp = rospy.Time.now()
		goal_pose.target_pose.pose.position.x = pose[0]
		goal_pose.target_pose.pose.position.y = pose[1]
		goal_pose.target_pose.pose.position.z = pose[2]
		goal_pose.target_pose.pose.orientation.x = pose[3]
		goal_pose.target_pose.pose.orientation.y = pose[4]
		goal_pose.target_pose.pose.orientation.z = pose[5]
		goal_pose.target_pose.pose.orientation.w = pose[6]

		return goal_pose


if __name__ == '__main__':
	rospy.init_node('nav_test', anonymous=False)
#	waypoints = np.genfromtxt('point.txt', dtype=float, delimiter=',')
	waypoints = np.genfromtxt('goal.txt', dtype=float, delimiter=',') 
	i = 0
	(x,y)  = waypoints.shape
	nav = Navigation()		
	while i < x:
		print (waypoints[i])
		nav.startNav(waypoints[i])
	 	i = i + 1

	rospy.spin()
