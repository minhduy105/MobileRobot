#!/usr/bin/env python

import argparse
import numpy as np 
import Image 
import math

def readImgToNp(name):
	im = Image.open(name)
	pix = np.array(im)
	return pix

#test to see whether the point is out of the map or in the wall
#if not return the normal map with the point in its and the coordinate of the point in c_space
def testPoint(pix,rad,point,name):
	(y,x) = pix.shape
	(xp,yp) = point
	subx = int(xp/rad)
	suby = int(yp/rad)
	if x < xp or y < yp:
		raise ValueError('The ' + name + ' position is not in the map.')
	
	endx = min([rad * (subx + 1), x])
	endy = min([rad * (suby + 1), y])
	box = pix[suby*rad:endy,subx*rad:endx]
	if np.amin(box) == 0:
		raise ValueError('The ' + name + ' position is in the wall')
	if name == 'start':
		pix[int(yp)][int(xp)]=50
	else:
		pix[int(yp)][int(xp)]=100
	return (pix,(subx,suby))

#map the normal map into c_space
def mapToCSpace(pix,rad):
	(y,x) = pix.shape
	idx = 0
	idy = 0
	csp = []
	while idy < y:
		idx = 0
		row = []
		while idx < x:
			box = pix[idy:min([idy + rad, y]),idx:min([idx + rad, x])]
			row.append(np.amin(box))
			idx = idx + rad
		csp.append(row)
		idy = idy + rad 
	csp = np.asarray(csp)

	#testing
	return csp

def searchPath(csp,start):
	return

def findPath(pix,rad,start,goal):
	try:
		rad = int(math.ceil(rad))
		(y,x) = pix.shape #note the array return row and then collumn
		binX = int(x/rad)
		binY = int(y/rad)
		if binX == 0 or binY == 0:
			raise ValueError('Radius is too large')
		# for testing if start or goal in the wall
		(pix,csta) = testPoint(pix,rad,start,"start")
		(pix,cend) = testPoint(pix,rad,goal,"goal")
		saveNpToImg(pix,'normal_map_with_start_and_goal.png')		

		csp = mapToCSpace(pix,rad)
		saveNpToImg(csp,'c_space_map_with_start_and_goal.png')		

		searchPath(csp,csta)
		return pix
			

	except ValueError as err:
		print 'Value Error: ', err

def saveNpToImg(data,namefile):
	img = Image.fromarray(data,'L')
	img.save(namefile)


if __name__ == '__main__':

	print ("Note: the map units is in meter. \n")
	parser = argparse.ArgumentParser(description='Plan a path in a map')
	parser.add_argument('--map', dest='mapfile')
	parser.add_argument('--radius', dest='radius')
	parser.add_argument('--start', dest='start', nargs=2)
	parser.add_argument('--goal', dest='goal', nargs=2)
	
	args = parser.parse_args()

	mapfile = args.mapfile
	radius = float(args.radius)
	start = tuple(map(float, args.start))
	goal = tuple(map(float, args.goal))
	

	pix = readImgToNp(mapfile)
	(y,x) = pix.shape
	print 'Map file:', mapfile
	print ('Map size: ' + str(x) + 'X' + str(y) + ' m^2')
	print ('Radius:' + str(radius) + ' m')
	print 'Start:', start
	print 'Goal:', goal

	pix = findPath(pix,radius,start,goal)

#	saveNpToImg(pix,'test2.png')	

	
