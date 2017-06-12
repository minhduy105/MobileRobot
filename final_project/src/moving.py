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

#NOTE: the id can only be one letter or number

class Scheduler():
	def __init__(self):
		self.starpoint = [0.455,1.590,0,0,0,-0.672,0.741] 
		self.in_motion = False
		self.continous_mode = False
		self.home_requested = 0
		self.get_to_location = False
		self.get_name = False
		self.id = 'None'
		self.loadPerson()
		# Mtion related variables

		self.move_base = actionlib.SimpleActionClient('move_base',MoveBaseAction)
		self.move_base.wait_for_server(rospy.Duration(5))
		self.goal = MoveBaseGoal()



		key_sub = rospy.Subscriber('keys', String, self.keyCallback)
		name_sub = rospy.Subscriber('/delivery_to',String, self.personCallback)
#		image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callbackImg)



	
	def loadPerson(self):
		self.person ={}
		with open('People.csv', mode='r') as infile:
			reader = csv.reader(infile)
			for rows in reader:
				wayspoint = [rows[1],rows[2],rows[3],rows[4],rows[5],rows[6],rows[7],rows[8]]
				self.person[rows[0]] = wayspoint
		print (self.person)
	
	def keyCallback(self, keys):
		#have get_name there so it do not do anythings at home 
		if keys.data == "h":
			self.goTo(self.starpoint)
		if self.get_name and self.get_to_location and not self.in_motion:
			self.id = keys.data
			print (self.id)

	def personCallback(self, word):
		self.get_name = True
		self.name = str(word.data)
		waypoints = copy.deepcopy(self.person[self.name.upper()])
		waypoints.pop(0)
		print (word)
		print (waypoints)
		self.goTo(waypoints)


	# def callbackImg(self,image):
	# 	if self.get_to_location and not self.in_motion:
	# 		i = 0
	# 		try:
	# 			cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
	# 		except CvBridgeError as e:
	# 			print(e)

	# 		i = self.waitForInput(cv_image)	
	# 		#say hey, get your stuff, yaa, and remember to write your id


	# 		if not self.in_motion and i > 30: #assume 30fps and see the face for 1 second without enter ID
	# 			#say yall, enter the id please
	# 			self.time_at_action = rospy.Time.now().secs  
	# 			i = self.waitForInput(cv_image)	
	# 			if not self.in_motion: #nothing happens
	# 				self.goTo(self.home_waypoints) #get home_waypoints
	# 				self.get_name = False #reset


	# def waitForInput(self,cv_image):
	# 	while rospy.Time.now().secs - self.time_at_action < 11:
	# 		faces = self.detectFace(cv_image)	
	# 		if not faces: 
	# 			i = i + 1
	# 		if self.id is not 'None':
	# 			if self.id == self.person[self.name.upper()][0]
	# 				rospy.sleep(Duration(5)) #wait 5 seconds for the person to go away
	# 				self.goTo(self.home_waypoints) #get home_waypoints
	# 				self.get_name = False #reset
	# 				break
	# 			else:


	# 				#take 5 pictures


	# 				self.goTo(self.home_waypoints) #get home_waypoints
	# 				self.get_name = False #reset
	# 			self.id = 'None' #reset
	# 		return i 		



	
	
	def goTo(self,waypoints):
		self.get_to_location = False
		self.in_motion = True        
				
		
		self.goal.target_pose.header.frame_id= 'map'
		self.goal.target_pose.header.stamp = rospy.Time.now()

		self.goal.target_pose.pose.position.x = float(waypoints[0])
		self.goal.target_pose.pose.position.y = float(waypoints[1])
		self.goal.target_pose.pose.position.z = float(0.0)
		
		self.goal.target_pose.pose.orientation.x = float(0.0)
		self.goal.target_pose.pose.orientation.y = float(0.0)
		self.goal.target_pose.pose.orientation.z = float(waypoints[5])
		self.goal.target_pose.pose.orientation.w = float(waypoints[6])
		
		print("Traveling to waypoint with pose")
		print(self.goal)
		self.move_base.send_goal(self.goal, self.goalComplete)
		print("Goal sent")
	
	def goalComplete(self,a,b):
		print("Goal Completed")
		self.in_motion = False
		self.get_to_location = True
		self.time_at_action = rospy.Time.now().secs



	def cancel(self):
		print("cancelling")
		self.move_base.cancel_all_goals()

	def home(self):
		print("Going home next")
		self.home_requested = True

	def detectFace(self,img):
		face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
		eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')
		
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		faces = face_cascade.detectMultiScale(gray,
					scaleFactor=1.1,
					minNeighbors=5,
					minSize=(30, 30),
					flags=cv2.cv.CV_HAAR_SCALE_IMAGE)


		#take 5 pictures



		for (x,y,w,h) in faces:
			cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
			roi_gray = gray[y:y+h, x:x+w]
			roi_color = img[y:y+h, x:x+w]
			eyes = eye_cascade.detectMultiScale(roi_gray)
			for (ex,ey,ew,eh) in eyes:
				cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
		return faces			



if __name__ == '__main__':
	rospy.init_node('moving')
	
	scheduler = Scheduler()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
