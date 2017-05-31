#!/usr/bin/env python


from random import randint, random, gauss
import numpy as np
import matplotlib.pyplot as plt; plt.rcdefaults()
import matplotlib.pyplot as plt
import math

class DiscreteCorridor:
	def __init__(self, length=10, door_positions=(2, 4, 7),
				 initial_position = 5):
		self.length = length
		self.door_positions = door_positions
		self.robot_position = initial_position

		self.movement_accuracy = 0.9
		self.sensor_accuracy = 0.9
		
	def __str__(self):
		corridor = ''
		for i in xrange(self.length):
			if i in self.door_positions:
				corridor += 'D'
			else:
				corridor += '_'
		corridor += '\n'
		corridor += ' ' * self.robot_position + 'R'
		return corridor
	
	def randomize_position(self):
		self.robot_position = randint(1, self.length) - 1

	def move_robot(self, move):
		self.robot_position += self.motion_model(move)
		self.robot_position = min(max(0, self.robot_position), self.length - 1)

	def sense_door(self):
		return self.sensor_model(self.robot_position in self.door_positions)
		
	def motion_model(self, movement):
		if random() > self.movement_accuracy:
			return movement + randint(0, 1) * 2 - 1
		else:
			return movement 

	def sensor_model(self, measurement):
		if random() > self.sensor_accuracy:
			return not measurement
		else:
			return measurement
		

class ContinuousCorridor:
	def __init__(self, starting_position=0.0, wall_position=10.0,
				 sensor_stdev=0.05, movement_stdev=0.01,
				 sensor_max = 20.0):
		self.position = starting_position
		self.wall = wall_position
		self.sensor_stdev = sensor_stdev
		self.move_stdev = movement_stdev
		self.sensor_max = sensor_max
		
	# Return distance measurement to the end of the corridor.  Clip it
	# so that there are no negative readings.
	def measurement(self):
		return min(max(self.wall - self.position + gauss(0.0, self.sensor_stdev), 0.0), self.sensor_max)

	# Move up the corridor.  Stop when you hit the wall.
	def move(self, u):
		self.position += u + gauss(0.0, self.move_stdev)
		if self.position > self.wall:
			self.position = self.wall
			

def drawGraph(bel):
	y_pos = np.arange(len(bel.tolist()))
	plt.bar(y_pos, bel.tolist(), align = 'center')
	plt.title('Graph')
	plt.savefig('graph.png')
	plt.gcf()
	plt.close()

def shiftProbabilityMap(gridMove,propInMap,bel):
	# shift the probability 	
	if gridMove > 0:
		propInMap = np.pad(propInMap,((0,0),(gridMove,0)), mode='constant')[:, :(-1*gridMove)]
		bel = np.pad(bel,((gridMove,0)), mode='minimum')[ :(-1*gridMove)]
	else:
		propInMap = np.pad(propInMap,((0,0),(0,(-1*gridMove))), mode='constant') [:,(-1*gridMove): ]
		bel = np.pad(bel,((0,(-1*gridMove))), mode='minimum') [(-1*gridMove): ]
	return (propInMap,bel)	

def printInforOfTheBot(corridorTrue,corridorGlobal):
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

def initializeMakov(corridorTrue):
	propInMap = np.zeros((len(corridorTrue.door_positions),corridorTrue.length))
	checkMap = []
	j = 0
	for i in corridorTrue.door_positions:
		propInMap[j][i] = 1.000
		poMap = DiscreteCorridor(length = corridorTrue.length, 
								door_positions= corridorTrue.door_positions,
								initial_position = i)
		poMap.movement_accuracy = 2.0		
		poMap.sensor_accuracy = 2.0
		checkMap.append(poMap)
		j = j + 1
	return (propInMap,checkMap)	


def MakovLocalization (corridorTrue,corridorGlobal):
	lookForDoor = False
	(propInMap,checkMap) = initializeMakov(corridorTrue)
	bel = np.ones((corridorTrue.length)) *  0.0001

	while True:
		(corridorTrue,corridorGlobal,gridMove,senseTrue) = printInforOfTheBot(corridorTrue,corridorGlobal)
		if lookForDoor:
			j = 0 
			out = []
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
			(propInMap,bel) = shiftProbabilityMap(gridMove,propInMap,bel)
			if propInMap.size > 0:
				bel = bel + np.sum(propInMap, axis =0) 	
			else: 
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
