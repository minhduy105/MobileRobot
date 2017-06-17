#!/usr/bin/env python

# Every python controller needs these lines
import rospy
import math
# The velocity command message
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
class MovingForward(): 

	def moving (self):

		rospy.init_node('move')
		rospy.Subscriber('/scan',LaserScan, self.callback)

	def callback(self,data):

		pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 10) 


		k = 0.1 #constant proportion value for proportional stop and tanh stop
		vel = 0.1 #initial velocity of the turtle
		stopAt = 0.25
		calSize = int(math.ceil(math.atan((0.354/2)/stopAt)/0.0017))
		
		command = Twist()
		dist = np.nanmin(np.asarray(data.ranges))

		print (dist)
		
		# if dist > self.stopD:
		# 	command.linear.x = vel
		# else: 
		# 	if self.stopType == "step":
		# 		if stopD == dist:
		# 			command.linear.x = 0.0
		# 		else:
		# 			command.linear.x = -1*vel
		# 	elif self.stopType == "stepDead":
		# 		command.linear.x = 0.0
		# 	elif self.stopType == "prop":
		# 		command.linear.x = k*dist*command.linear.x
		# 	elif self.stopType == "tanh":
		# 		command.linear.x = math.tanh(k*dist)*command.linear.x
		
		# print (dist)
		# command.linear.y = 0.0
		# command.linear.z = 0.0
		# command.angular.x = 0.0
		# command.angular.y = 0.0
		# command.angular.z = 0.0

		# # Loop at 10Hz, publishing movement commands until we shut down.
		# rate = rospy.Rate(10)
		# pub.publish(command)
		#rate.sleep()
			
if __name__ == '__main__':
	#get the parameter value
	
	# stopD = 1.0
	# stopType = "tanh"

	#0.354 = width of the robot,  0.0031088677301 = angle increment (total both size) 
	#calculating the width of the turtle bot
	#do this outside because it take a lot of compute power

	#-------For neato Robot--------#
	#calSize = int(math.ceil(math.atan((0.354/2)/stopD)/0.017))

	x = MovingForward()
	x.moving()
	rospy.spin()
