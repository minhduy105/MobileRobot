#!/usr/bin/env python

from random import randint, random, gauss, sample
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
			
