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
	print (name + ' positition in c-space is: ' + str(subx) + ', '+ str(suby))
		
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
	(y,x) = csp.shape
	(xs,ys) = start
	copyMap = np.copy(csp)
	copyMap[copyMap == 0] = 1
	copyMap[copyMap == 255] = 0
	copyMap[copyMap == 50] = 3
	copyMap[copyMap == 100] = 2
	findGoal = True

	idx = []
	idx.append((ys,xs))

	#forward
	goal = (-1,-1) # can't find it
	while idx:
		(iy,ix) = idx[0]
		idx.pop(0)
		
		if copyMap[max([0,iy-1])][ix] == 2: #up
			copyMap[max([0,iy-1])][ix] = copyMap[iy][ix] + 1
			goal = (max([0,iy-1]),ix)
			break
		elif copyMap[max([0,iy-1])][ix] == 0:
			copyMap[max([0,iy-1])][ix] = copyMap[iy][ix] + 1
			idx.append((max([0,iy-1]),ix))

		if copyMap[min([y-1,iy+1])][ix] == 2: #down
			copyMap[min([y-1,iy+1])][ix] = copyMap[iy][ix] + 1
			goal = (min([y-1,iy+1]),ix)
			break
		elif copyMap[min([y-1,iy+1])][ix] == 0:
			copyMap[min([y-1,iy+1])][ix] = copyMap[iy][ix] + 1
			idx.append((min([y-1,iy+1]),ix))

		if copyMap[iy][max([0,ix-1])] == 2: #left
			copyMap[iy][max([0,ix-1])] = copyMap[iy][ix] + 1
			goal = (iy,max([0,ix-1]))
			break
		elif copyMap[iy][max([0,ix-1])] == 0:
			copyMap[iy][max([0,ix-1])] = copyMap[iy][ix] + 1
			idx.append((iy,max([0,ix-1])))

		if copyMap[iy][min([x-1,ix+1])] == 2: #right
			copyMap[iy][min([x-1,ix+1])] = copyMap[iy][ix] + 1
			goal = (iy,min([x-1,ix+1]))
			break
		elif copyMap[iy][min([x-1,ix+1])] == 0:
			copyMap[iy][min([x-1,ix+1])] = copyMap[iy][ix] + 1
			idx.append((iy,min([x-1,ix+1])))	

	
	np.savetxt("result/goalFinding.csv", copyMap, delimiter=",")	
	if goal == (-1,-1):
		findGoal = False
		return (findGoal,csp,idx)

	#backward
	idx = []
	(yg,xg) = goal
	idx.append((yg,xg))
	(iy,ix) = idx[len(idx)-1]

	while copyMap[iy][ix] != 3 and idx:
		(iy,ix) = idx[len(idx)-1] #get the last element
		if iy-1 < 0:
			idx.append((iy-1,ix))
			continue
		elif copyMap[iy-1][ix] == copyMap[iy][ix] - 1: #up
			idx.append((iy-1,ix))
			continue

		if iy+1>y-1:
			idx.append((iy+1,ix))
			continue
		elif copyMap[iy+1][ix] == copyMap[iy][ix] - 1: #down
			idx.append((iy+1,ix))
			continue

		if ix-1 < 0:
			idx.append((iy,ix-1))
			continue	
		elif copyMap[iy][ix-1] == copyMap[iy][ix] - 1: #left
			idx.append((iy,ix-1))
			continue

		if ix+1 > x-1:
			idx.append((iy,ix+1))
			continue
		elif copyMap[iy][ix+1] == copyMap[iy][ix] - 1: #right
			idx.append((iy,ix+1))
			continue

	for i in idx:
		(y,x) = i
		csp[y][x] = 100

	saveNpToImg(csp,'c_space_map_with_path_to_goal.png')		
			
	return (findGoal,csp,idx)


def fromCSpaceToOri(pix,idx,rad):
	i = 0
	while i < len(idx) - 1:
		(y,x) = idx[i]
		y = y * rad
		x = x * rad

		(yn,xn) = idx[i + 1]
		yn = yn * rad
		xn = xn * rad

		if y == yn:
			if x < xn:
				pix[y,x:xn] = 100
			else:
				pix[y,xn:x] = 100
		elif x == xn:
			if y < yn:
				pix[y:yn,x] = 100
			else:
				pix[yn:y,x] = 100
		else:
			print ('oops,somethings is wrong')
		i = i + 1

	saveNpToImg(pix,'normal_map_with_path_to_goal.png')	
	return pix

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

		(findGoal,csp,idx) = searchPath(csp,csta)
		print ("The path in C-Space (from goal to start): ")
		print (idx)

		if findGoal:
			pix = fromCSpaceToOri(pix,idx,rad)
		else:
			print ("Cannot find the path")
		return pix
			

	except ValueError as err:
		print 'Value Error: ', err

def saveNpToImg(data,namefile):
	img = Image.fromarray(data,'L')
	img.save("result/"+namefile)


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

	
