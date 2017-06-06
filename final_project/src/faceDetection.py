#!/usr/bin/env python

from __future__ import print_function
import roslib; roslib.load_manifest('sound_play')
import sys
import rospy
import cv2
import rospy
import numpy as np
import pyocr
import pyocr.builders
import actionlib
import subprocess

from sensor_msgs.msg import Image
from std_msgs.msg import String
from PIL import Image as Img
from cv_bridge import CvBridge, CvBridgeError


class face_detection:

	def __init__(self):

#		self.name_pub = rospy.Publisher("delivery_to",String,queue_size=10)

		self.bridge = CvBridge()
		#this is for start searching "name" when the user hit s, and stop when it finds the name
		self.start = False 
		#this is the directory of all the person in the office with their ID
		self.person ={"Duy": 01,
					"Alison": 02}
		#set up the character recognition tool
		image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callbackImg)
		key_sub = rospy.Subscriber('keys', String,self.callbackStr)

	def getletter(self,img):
		txt = self.tool.image_to_string(
			img,
			lang=self.lang,
			builder=pyocr.builders.TextBuilder()
		)
		return txt

#new node 
	def sayStuff(self,word):
		cmd = ["rosrun sound_play say.py " + '"'+ word+'"']
		subprocess.Popen(cmd,shell=True)
	
	def detectFace(self,img):
		face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
		eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')
		
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		faces = face_cascade.detectMultiScale(gray,
					scaleFactor=1.1,
					minNeighbors=5,
					minSize=(30, 30),
					flags=cv2.cv.CV_HAAR_SCALE_IMAGE)
		print (faces)
		for (x,y,w,h) in faces:
			cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
			roi_gray = gray[y:y+h, x:x+w]
			roi_color = img[y:y+h, x:x+w]
			eyes = eye_cascade.detectMultiScale(roi_gray)
			for (ex,ey,ew,eh) in eyes:
				cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
		cv2.imshow('img',img)
		cv2.imshow('gray',gray)
			
		cv2.waitKey(3)

	def callbackImg(self,image):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
		except CvBridgeError as e:
			print(e)

		self.detectFace(cv_image)	


	def callbackStr(self,keys):
		word = keys.data
		if word == 's':
			self.start = True# start looking for the person
		else :
			print ("Hit s to start looking for name")

	
if __name__ == '__main__':
	rospy.init_node('detect_face', anonymous=True)
	ic = face_detection()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()










import numpy as np
import cv2
