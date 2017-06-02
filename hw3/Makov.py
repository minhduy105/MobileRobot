#!/usr/bin/env python

# Every python controller needs these lines
import rospy
import math
# The velocity command message
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
class MovingForward(): 

    def __init__(self,stopD,stopType,calSize):
        self.stopD = stopD
        self.stopType = stopType
        self.calSize = calSize

    def moving (self):

        rospy.init_node('move')
        rospy.Subscriber('/scan',LaserScan, self.callback)

#-------For neato Robot--------#
        # rospy.Subscriber('/base_scan',LaserScan, self.callback)
                

    def callback(self,data):

        pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 10) 

#-------For neato Robot--------#
#        pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10) 
        
        k = 0.1 #constant proportion value for proportional stop and tanh stop
        vel = 0.1 #initial velocity of the turtle
        
        calSize = self.calSize  
        
        command = Twist()
        #-------For neato Robot--------#
        # impInput1 = np.asarray(data.ranges[0:calSize])
        # impInput2 = np.asarray(data.ranges[len(data.ranges)-calSize:len(data.ranges)])
        # impInput = np.concatenate((impInput1,impInput2), axis=1)
        # dist = np.nanmin(impInput[np.nonzero(impInput)])

        impInput = np.asarray(data.ranges[len(data.ranges)/2-calSize:len(data.ranges)/2+calSize])
        dist = np.nanmin(impInput)
        
        if dist > self.stopD:
            command.linear.x = vel
        else: 
            if self.stopType == "step":
                if stopD == dist:
                    command.linear.x = 0.0
                else:
                    command.linear.x = -1*vel
            elif self.stopType == "stepDead":
                command.linear.x = 0.0
            elif self.stopType == "prop":
                command.linear.x = k*dist*command.linear.x
            elif self.stopType == "tanh":
                command.linear.x = math.tanh(k*dist)*command.linear.x
        
        print (dist)
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0

        # Loop at 10Hz, publishing movement commands until we shut down.
        rate = rospy.Rate(10)
        pub.publish(command)
        #rate.sleep()
            
if __name__ == '__main__':
	#get the parameter value
    stopD = rospy.get_param('/stopDistance')
    stopType = rospy.get_param('/stopType')

    # stopD = 1.0
    # stopType = "tanh"

    #0.354 = width of the robot,  0.0031088677301 = angle increment (total both size) 
    #calculating the width of the turtle bot
    #do this outside because it take a lot of compute power
    calSize = int(math.ceil(math.atan((0.354/2)/stopD)/0.0017))

    #-------For neato Robot--------#
    #calSize = int(math.ceil(math.atan((0.354/2)/stopD)/0.017))

    x = MovingForward(stopD,stopType,calSize)
    x.moving()
    rospy.spin()
from random import randint, random, gauss, sample
import numpy as np
import matplotlib.pyplot as plt; plt.rcdefaults()
import matplotlib.pyplot as plt
import math
from corridor import DiscreteCorridor, ContinuousCorridor

#-------------------------------------------My Code-----------------------------------------------------#
def drawGraph(bel): #draw the graph, there is some warning, just ignore them
	y_pos = np.arange(len(bel.tolist()))
	plt.bar(y_pos, bel.tolist(), align = 'center')
	plt.title('Graph')
	plt.savefig('graph.png')
	plt.gcf()
	plt.close()

def shiftProbabilityMap(gridMove,propInMap,bel): #move the probability with the whole screen
	# shift the probability 	
	if gridMove > 0:
		propInMap = np.pad(propInMap,((0,0),(gridMove,0)), mode='constant')[:, :(-1*gridMove)]
		bel = np.pad(bel,((gridMove,0)), mode='minimum')[ :(-1*gridMove)]
	else:
		propInMap = np.pad(propInMap,((0,0),(0,(-1*gridMove))), mode='constant') [:,(-1*gridMove): ]
		bel = np.pad(bel,((0,(-1*gridMove))), mode='minimum') [(-1*gridMove): ]
	return (propInMap,bel)	

def printInforOfTheBot(corridorTrue,corridorGlobal): #print the position related to the word, the map, and the door detection
	print 'Position before:', corridorGlobal.position
	#discritize it as 1 value (from 0 to 1 belong to 0 square)
	ini = int(math.floor(corridorGlobal.position)) 
	u = float(raw_input('Move? '))
	corridorGlobal.move(u)
	print 'Position after:', corridorGlobal.position
	aft = int(math.floor(corridorGlobal.position))
	gridMove = aft - ini
	print 'Total move in grid map: ',gridMove
	corridorTrue.move_robot(gridMove)
	print corridorTrue
	senseTrue = corridorTrue.sense_door()
	print 'Door:', senseTrue
	return (corridorTrue,corridorGlobal,gridMove,senseTrue)

def initializeMakov(corridorTrue): #initializeMakov
	propInMap = np.zeros((len(corridorTrue.door_positions),corridorTrue.length))
	checkMap = []
	j = 0
	for i in corridorTrue.door_positions: # door is the main feature to look at
		propInMap[j][i] = 1.000
		poMap = DiscreteCorridor(length = corridorTrue.length, 
								door_positions= corridorTrue.door_positions,
								initial_position = i)
		poMap.movement_accuracy = 2.0 #this is for calculate probability, assume it is perfect		
		poMap.sensor_accuracy = 2.0 #this is for calculate probability, assume it is perfect
		checkMap.append(poMap)
		j = j + 1
	return (propInMap,checkMap)	

def MakovLocalization (corridorTrue,corridorGlobal):
	lookForDoor = False #inital state of not seeing any door
	(propInMap,checkMap) = initializeMakov(corridorTrue)
	bel = np.ones((corridorTrue.length)) *  0.0001

	while True:
		#move the bot
		(corridorTrue,corridorGlobal,gridMove,senseTrue) = printInforOfTheBot(corridorTrue,corridorGlobal)
		if lookForDoor:
			j = 0 
			out = []
			#move the probability map
			for i in checkMap:
				i.move_robot(gridMove) #move the bot in all of the map
				senseMap = i.sense_door()
				if senseTrue != senseMap: #check if that map is correct
					out.append(j) #get the incorrect map

				j = j + 1
			if out:#delete all the incorrect map and its probability
				j = 0
				for i in out:
					if j > 0:
						i = i - j
					checkMap.pop(i)
					propInMap = np.delete(propInMap,i,0)
					j = j + 1
			#move the probability map		
			(propInMap,bel) = shiftProbabilityMap(gridMove,propInMap,bel)
			if propInMap.size > 0:
				bel = bel + np.sum(propInMap, axis =0) 	
			else:  #when all of the probability is incorrect, reset it
				lookForDoor = False
				(propInMap,checkMap) = initializeMakov(corridorTrue)
		
		else:
			if senseTrue:
				lookForDoor = True
				# for i in checkMap:
				# 	print i
				bel = bel + np.sum(propInMap, axis =0)

		bel = bel/np.sum(bel)
		drawGraph(bel)	

if __name__ == '__main__':
	# assume there is no duplicate in the door input
	corridorTrue = DiscreteCorridor()
	corridorTrue.randomize_position()
	print ("True Position:")
	print corridorTrue
	corridorGlobal = ContinuousCorridor(starting_position=corridorTrue.robot_position)
	MakovLocalization (corridorTrue,corridorGlobal)
