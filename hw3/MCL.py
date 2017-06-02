#!/usr/bin/env python

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

def NormalizeMCL(propInMap): #normalize the matrix
	np.clip(propInMap, 0, 10, out=propInMap) #make sure there is no negative and no too big value
	propInMap = propInMap/np.sum(propInMap)
	#print propInMap
	return propInMap 

def initializeMCL(corridorTrue, sizeOfSetPose):
	propInMap = np.ones((sizeOfSetPose,corridorTrue.length)) * 0.0001 #don't want it to be zero as it can cause some zero division later
	checkMap = []
	PoseSet = sample(range(1, corridorTrue.length), sizeOfSetPose) #get the set of pose using sample without replacement 
	j = 0
	for i in PoseSet:
		propInMap[j][i] = 1.0001 #the location of the pose in the map
		poMap = DiscreteCorridor(length = corridorTrue.length, 
								door_positions= corridorTrue.door_positions,
								initial_position = i)
		poMap.movement_accuracy = 2.0		
		poMap.sensor_accuracy = 2.0
		checkMap.append(poMap)
		
		# print poMap
		# print propInMap[j]
	
		j = j + 1
	propInMap = NormalizeMCL(propInMap)
	return (propInMap,checkMap)	

def updateAndSortMapPropFromSensor(senseTrue,propInMap,checkMap): #update the map with sensor data and sort the map in decensding order
	j = 0 
	for i in checkMap:
		senseMap = i.sense_door()
		if senseTrue != senseMap: #decrease the prop if the sensor is not match
			propInMap[j] = propInMap[j] / 2.0
		else : #increase the prop if the sensor is match
			propInMap[j] = propInMap[j] * 3.0
		j = j + 1
	#sorting below
	totalProp = np.sum(propInMap,axis = 1) #get the total probability of each location
	idx = totalProp.argsort()[::-1] #sort its index
	propInMap = np.take(propInMap,idx,axis=0) #sort the array
	checkMap = [x for (y,x) in sorted(zip(totalProp,checkMap),reverse=True)] #sort the represent map
	propInMap = NormalizeMCL(propInMap)

	# print "Original"
	# j = 0
	# for i in checkMap:
	# 	print i
	# 	print propInMap[j]
	# 	j = j + 1

	return (propInMap,checkMap)

def deleteLowPropPose(propInMap,checkMap): #delete the probability that is smaller than the thresh hold
	(x,y) = propInMap.shape
	bound = 1.0 / x #the thresh hold
	totalProp = np.sum(propInMap,axis = 1)
	i = x-1 #it is decending order, so go from the end of the list
	while i >= 0:
		if totalProp[i] < bound:
			checkMap.pop(i)
			propInMap = np.delete(propInMap,i,0)	
		else:
			break
		i = i - 1

	# print "delete"
	# j = 0
	# for i in checkMap:
	# 	print i
	# 	print propInMap[j]
	# 	j = j + 1
	
	return (propInMap,checkMap)	

def getNewPose(propInMap,checkMap,sizeOfSetPose,corridorTrue): #add the new pose into our set of pose
	(x,y) = propInMap.shape
	if x == sizeOfSetPose:
		return (propInMap,checkMap)
	else: #add pose
		PoseSet = sample(range(1, corridorTrue.length), sizeOfSetPose - x)
		for i in PoseSet:

			propMap = np.ones(corridorTrue.length) * np.amin(propInMap)
			propMap[i] = np.amin(propInMap) + (np.amax(propInMap) - np.amin(propInMap)) / 3.0 #get the median value
			poMap = DiscreteCorridor(length = corridorTrue.length, 
									door_positions= corridorTrue.door_positions,
									initial_position = i)
			poMap.movement_accuracy = 2.0		
			poMap.sensor_accuracy = 2.0
			propInMap = np.append(propInMap, [propMap], axis=0)
			checkMap.append(poMap)
		

	# print "update"
	# j = 0
	# for i in checkMap:
	# 	print i
	# 	print propInMap[j]
	# 	j = j + 1	
	
	propInMap = NormalizeMCL(propInMap)
	return (propInMap,checkMap)	

def shiftMap(propInMap,checkMap,gridMove): #shift the probability
	for i in checkMap:
		i.move_robot(gridMove)	
	if gridMove > 0:
		propInMap = np.pad(propInMap,((0,0),(gridMove,0)), mode='minimum')[:, :(-1*gridMove)]
	else:
		propInMap = np.pad(propInMap,((0,0),(0,(-1*gridMove))), mode='minimum') [:,(-1*gridMove): ]

	# print "shift"
	# j = 0
	# for i in checkMap:
	# 	print i
	# 	print propInMap[j]
	# 	j = j + 1	
	
	propInMap = NormalizeMCL(propInMap)
	return (propInMap,checkMap)		

def MCL(corridorTrue,corridorGlobal):
	sizeOfSetPose = 8
	(propInMap,checkMap) = initializeMCL(corridorTrue,sizeOfSetPose)
	senseTrue = corridorTrue.sense_door()
	(propInMap,checkMap) = updateAndSortMapPropFromSensor(senseTrue,propInMap,checkMap)
	(propInMap,checkMap) = deleteLowPropPose(propInMap,checkMap)

	while True:
		(propInMap,checkMap) = getNewPose(propInMap,checkMap,sizeOfSetPose,corridorTrue)		
		(corridorTrue,corridorGlobal,gridMove,senseTrue) = printInforOfTheBot(corridorTrue,corridorGlobal)
		(propInMap,checkMap) = shiftMap(propInMap,checkMap,gridMove)
		(propInMap,checkMap) = updateAndSortMapPropFromSensor(senseTrue,propInMap,checkMap)
		(propInMap,checkMap) = deleteLowPropPose(propInMap,checkMap)
		bel = np.sum(propInMap, axis =0)
		bel = bel/np.sum(bel)		
		drawGraph(bel)

if __name__ == '__main__':

	# assume there is no duplicate in the door input
	
	corridorTrue = DiscreteCorridor()
	corridorTrue.randomize_position()
	print ("True Position:")
	print corridorTrue
	corridorGlobal = ContinuousCorridor(starting_position=corridorTrue.robot_position)
	MCL(corridorTrue,corridorGlobal)