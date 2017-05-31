import message_filters
from sensor_msgs.msg import LaserScan, CameraInfo
from nav_msgs.msg import Odometry
import rospy
import numpy as np 
import math
from numpy import sign
import Image

class DrawMap(): 

	def __init__(self):
		self.gridMap = np.zeros((200,200))
		self.im = Image.new('L', (200, 200))
	
		return

	def draw (self):

		rospy.init_node('draw')
		laser_sub = message_filters.Subscriber('scan', LaserScan)
		odo_sub = message_filters.Subscriber('base_pose_ground_truth', Odometry)
		 
		ts = message_filters.TimeSynchronizer([laser_sub, odo_sub], 10)
		ts.registerCallback(self.callback)

	def covered_cells(self,start, end): #get it from Bill

		x_range = end[0] - start[0]
		y_range = end[1] - start[1]

		# If the start and the end are the same point, then we don't do
		# anything.
		if x_range == 0 and y_range == 0:
			return
			yield

		# Step through x or y?  Pick the one with the longer absolute
		# range.
		if abs(x_range) > abs(y_range):
			y_step = float(y_range) / abs(float(x_range))
			y = float(start[1])
			for x in xrange(start[0], end[0], sign(x_range)):
				yield((x, int(round(y))))
				y += y_step
		else:
			x_step = float(x_range) / abs(float(y_range))
			x = float(start[0])
			for y in xrange(start[1], end[1], sign(y_range)):
				yield((int(round(x)), y))
				x += x_step



	def callback(self, laser_data, odo_data):
		xy_laser = np.ones((3,640))
		i = 0

		#creating matrix from to convert scan data to odometry
		theta  = math.atan2(odo_data.pose.pose.orientation.z, odo_data.pose.pose.orientation.w) * 2
		tfToGround = np.zeros((3,3))
		tfToGround[0][0] = math.cos(theta)
		tfToGround[1][0] = math.sin(theta)
		tfToGround[0][1] = -math.sin(theta)
		tfToGround[1][1] = math.cos(theta)
		tfToGround[0][2] = odo_data.pose.pose.position.x
		tfToGround[1][2] = odo_data.pose.pose.position.y
		tfToGround[2][2] = 1

		#get x,y laser contacts data in robot coordinate frame
		while i < 640:
			angle = i*laser_data.angle_increment + laser_data.angle_min
			xy_laser[0][i] = laser_data.ranges[i]*math.cos(angle)
			xy_laser[1][i] = laser_data.ranges[i]*math.sin(angle)
			i = i + 1

		#get x,y laser contacts data in global frame
		trueDist = np.dot(tfToGround,xy_laser) 

		#convert the value in global frame to the grid value
		tfToGrid = np.zeros((2,3))
		tfToGrid[0][0] = 15
		tfToGrid[1][1] = 15

		gridDist = np.around(np.dot(tfToGrid,trueDist)).astype(int)
		#this is the position of the robot in the grid value
		gridPos = np.around(np.dot(tfToGrid,tfToGround[:,2])).astype(int)

		i = 0
		while i < 640:		
			self.gridMap[gridDist[0][i]][gridDist[1][i]] = self.gridMap[gridDist[0][i]][gridDist[1][i]] + 3

			for (x,y) in self.covered_cells((gridDist[0][i],gridDist[1][i]),(gridPos[0],gridPos[1])):
				self.gridMap[x][y] = self.gridMap[x][y] - 1
			i = i + 1

		np.clip(self.gridMap, 0, 15, out=self.gridMap)		
		self.im.putpixel((100, 50), 255)
		self.im.putdata([x for row in self.gridMap for x in row], 16, 0)
		self.im.save('drawedmap.jpg')

			
if __name__ == '__main__':

	x = DrawMap()
	x.draw()
	rospy.spin()


