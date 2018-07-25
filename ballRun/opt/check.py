from __future__ import division

from multimethod import multimethod

import os
import subprocess
import sys
import time
import glob

import numpy as np
import scipy
from scipy.optimize import differential_evolution, basinhopping
#import cma
import math

#from PIL import Image, ImageDraw
#from optimize import Obstacle



BALL_DIAMETER = 0.017270193333
DIMENSION_X = 0.287351820006
DIMENSION_Y = 0.01
DIMENSION_Z = 0.0425280693609

class Obstacle():

	def __init__(self):
		self.dimensions = []
		self.pos = []
		self.angle = 0.0

def get_init_setup():

	obst = []
  	#print(str(x))
	#f1 = open("./params/dyn_setup.txt", 'r')
	#f1 = open("./tests/tofix/best_from_select.txt", 'r')
	f1 = open("./tests/best_from_select.txt", 'r')
	#l = f1.readlines()
	#print(len(l))
  	#for i in range(0, len(l) - 3):
	lines = f1.readlines()
	for line in lines:
		#print line
		#if not line.strip():
	
			#print i 
		cont = line.split(' ')
		
		#print cont
		if len(cont) == 8:
  			o = Obstacle()
  			o.dimensions.append(float(cont[1]))
  			o.dimensions.append(float(cont[2]))
  			o.dimensions.append(float(cont[3]))
  			o.pos.append(float(cont[-4]))
  			o.pos.append(float(cont[-3]))
  			o.pos.append(float(cont[-2]))
 			o.angle = float(cont[-1])

  			obst.append(o)

	f1.close()
  	return obst 


def get_valid_output(obst):

	f1 = open('./tests/what_did_you_get.txt', 'w')

	for o in obst:
		f1.write("o ")
		f1.write(str(o.dimensions[0]) + ' ' + str(DIMENSION_Y) + ' ' + str(o.dimensions[2]) + ' ')
		        	#print o.pos[0][0]
		f1.write(str(float(o.pos[0])) + ' 0.0 ' + str(float(o.pos[2])) + ' ' + str(float(o.angle))+'\n' )

	f1.close()

	movable = []
	fixed = []	
	if len(obst) == 7 :
		# one fixed obstacle
		movable = obst[:3]
		
		fixed = obst[-4:]
	else :
		movable = obst[:3]
		
		fixed = obst[-3:]
		

	#print movable
  	#print fixed

	# from left to right in x direction
	movable.sort(key = lambda x: x.pos[0])
	#fixed.sort(key = lambda x: x.pos[0])
	
	#o1 = movable[0]
	#o2 = movable[1]
	
	#o3 = None
	#o3_ver = []

	collide = set([])
	
	#if (len(movable) > 2):
	#	o3 = movable[2]

	cmd = "./track".split()
	setup_path = './tests/what_did_you_get.txt'
	subprocess.call(cmd + [str(setup_path)], shell= False)
	f = open("./tests/track.txt", 'r')

	lines = f.readlines()

	#o2_ver = makeRectangle(o2.dimensions[0], o2.dimensions[2], o2.angle, (o2.pos[0], o2.pos[2]))
	#o1_ver = makeRectangle(o1.dimensions[0], o1.dimensions[2], o1.angle, (o1.pos[0], o1.pos[2]))
	#if(o3):
	#	o3_ver = makeRectangle(o3.dimensions[0], o3.dimensions[2], o3.angle, (o3.pos[0], o3.pos[2]))
	#f = glob.glob()
	#for f in files:
	os.remove('./tests/hit_track.txt')
	f2 = open('./tests/hit_track.txt', 'a')

	counter = []

	for i in range(len(movable)):
		counter.append(0)

	for l in lines:
		track = l.split()
		pos_x = float(track[0])
		pos_z = float(track[1])

		if(len(collide) == len(movable)):
			break

		for j, o in enumerate(movable) :
			ver = makeRectangle(o.dimensions[0], o.dimensions[2], o.angle, (o.pos[0], o.pos[2]))
			#f2.write(str(ver) + '\n')
			#f2.write(str(pos_x) + " " + str(pos_z) + '\n')
			# top-right, right_under, left_under, top_left
			#f2.write(str(ver) + '\n')
			#f2.write(str(ver[0]) + '\n')
			
			top = under = ver[0][1] 
			left =right = ver[0][0] 
			
			for v in ver:
				if v[0] < left:
					left = v[0]
				if v[0] > right :
					right = v[0]
				if v[1] < under :
					under = v[1]
				if v[1] > top:
					top = v[1]    
			'''                  
			copy = ver
			copy.sort(key = lambda x: x[1])
			top = copy[-1][1]
			under = copy[0][1]

			copy.sort(key = lambda x: x[0])
			left = copy[0][0]
			right = copy[-1][0]
			'''
			#print ver
			#print pos_x, pos_z
			
			if(pos_x >= (left- BALL_DIAMETER)  and pos_x <= (right + BALL_DIAMETER) and pos_z >= (under - BALL_DIAMETER) and pos_z <= (top+ BALL_DIAMETER)):
				#print "colliding with movable %1d"%movable.index(o)
				f2.write("colliding with movable %1d"%movable.index(o) + '\n')
				f2.write(str(ver) + '\n')
				#print ver
				#print pos_x, pos_z

				#d = math.sqrt((pos_x - o.pos[0])**2 + (pos_z - o.pos[2])**2)
				#r = math.sqrt(o.dimensions[0]**2 + o.dimensions[2]**2)
				#if(d < r):
				'''
					print "colliding with movable %1d"%movable.index(o)
					print r
					print d
					print pos_x, pos_z
					print o.pos[0], o.pos[2]
					collide.add(o)
					#collide.add(o)
				'''
				#print "in range from movable %1d"%movable.index(o)
				#d0 = math.sqrt((pos_x - ver[0][0])**2 + (pos_z - ver[0][1])**2)
				#d1 = math.sqrt((pos_x - ver[1][0])**2 + (pos_z - ver[1][1])**2)
				#d2 = math.sqrt((pos_x - ver[2][0])**2 + (pos_z - ver[2][1])**2)
				#d3 = math.sqrt((pos_x - ver[3][0])**2 + (pos_z - ver[3][1])**2)
			
				#ds = [d0,d1,d2,d3]
				ds = []
				p = (pos_x, pos_z)
				#ver.sort(key = lambda x: x[0])
					
				for i in range(0,len(ver)):
			
					ind2 = (i + 1)% len(ver)
					p1 = ver[i]
					p2 = ver[ind2]

					f2.write(str(p1) +  ' ' + str(p2) +'\n')
					d =  distance(p, p1, p2)
					f2.write(str(d) +'\n')
					#ds.append( np.linalg.norm( np.cross(np.subtract( p2 , p1), np.subtract(p1 , p)))/np.linalg.norm(np.subtract(p2 , p1)))
					ds.append(d)
				#arr = np.array(ds)
				#arr.sort()
				d = min(ds)
				#ind_1 = ds.index(arr[0])
				#ind_2 = ds.index(arr[1])
			
				
				#p2 = ver[ind_2]
				#p1 = ver[ind_1]
				#d = np.cross(np.subtract( p2 , p1), np.subtract(p1 , p))/np.linalg.norm(np.subtract(p2 , p1))

				#d = distance(p, p1, p2)
				#print arr
				#print "ind 1, ind 2"
				#print ind_1, ind_2 
				#print d
				f2.write(str(ds) + '\n')
				f2.write(str(d) + '\n')
				f2.write(str(p) + '\n')
				#collide.add(o)
				#print p1, p2
				#f2.write(str(p1) + ' ' + str(p2) + '\n')
				#if (d < BALL_DIAMETER):
				if (d < BALL_DIAMETER + BALL_DIAMETER/2.0):
					#f2.write("colliding with movable %1d"%movable.index(o) + '\n')
					#print "colliding with movable %1d"%movable.index(o)
					#print ver
					
					
					#print d
					#print p
					#print o.pos[0], o.pos[2]
					#collide.add(o)
					counter[j] += 1
			
	
	for i, c in enumerate(counter):
		if c > 5:
			collide.add(movable[i])


	f.close()
	f2.close()
	# before everything changed, identify which obstacle is idle
	idle = []
	idle_index = []

	for o in movable:
		if not o in collide:
			idle.append(o)
			#idle_index.append(movable)
	coll = []

	for i, o in enumerate( collide):
		print ("collide %1d"%i, o.pos[0] )
		coll.append(o)

	for o in idle:
		print ("idle %1d"%idle.index(o), o.pos[0] )

	coll.sort(key = lambda x: x.pos[0])
	
	right = fixed[-2]
	left = fixed[-3]

	if (right.pos[0] < left.pos[0]):
		tmp = right
		right = left
		left = tmp

	if (len(idle) > 0):
		
		ver_right = makeRectangle(right.dimensions[0], right.dimensions[2], right.angle, (right.pos[0], right.pos[2]))
		ver_left = makeRectangle(left.dimensions[0], left.dimensions[2], left.angle, (left.pos[0], left.pos[2]))
				
		if (len(idle) > 2):
		# we dont need that much neither
			for i in range(2, len(idle)):
				idle[i].pos[0] = 10

			idle[0] = move_idle_right(idle[0], right)
			idle[1] = move_idle_left(idle[1], left) 
			i = idle[0]
			j = idle[1]
			ver_0 = makeRectangle(i.dimensions[0], i.dimensions[2], i.angle, (i.pos[0], i.pos[2]))
			ver_1 = makeRectangle(j.dimensions[0], j.dimensions[2], j.angle, (j.pos[0], j.pos[2]))
			
			if len(fixed) > 3:
				for f in fixed:
					ver_f =  makeRectangle(f.dimensions[0], f.dimensions[2], f.angle, (f.pos[0], f.pos[2]))
					if overlap(ver_0, ver_f):

						idle[0].pos[0] = 0.5
						idle[0].pos[2] = 0.5	
						#idle[0].angle = idle[0].angle/2.0
						#idle[0] = move_idle_right(idle[0], right)
					if(overlap(ver_1, ver_f)):
						#idle[1].angle = idle[1].angle/2.0
						#idle[1] = move_idle_left(idle[1], left)
						idle[1].pos[0] = 0.5
						idle[1].pos[2] = 0.5 
		
		elif(len(idle) == 1):
			
			i = idle[0]
			
			#print ver_right
			i = move_idle_right(i, right)
			

			if len(fixed) > 3:
				ver = makeRectangle(i.dimensions[0], i.dimensions[2], i.angle, (i.pos[0], i.pos[2]))
				for f in fixed:
					ver_f =  makeRectangle(f.dimensions[0], f.dimensions[2], f.angle, (f.pos[0], f.pos[2]))
					if overlap(ver, ver_f):
						i.angle = idle[0].angle/2.0
						i = move_idle_right(i, right)
					
	
			idle[0] = i
		
			
		elif(len(idle) == 2):

			#i = idle[0]
			#i2 = idle[1]

			i = move_idle_right(idle[0], right)
			j = move_idle_left(idle[1], left) 
			#if(idle[0].angle > 0 ):
			if len(fixed) > 3:
				ver_0 = makeRectangle(i.dimensions[0], i.dimensions[2], i.angle, (i.pos[0], i.pos[2]))
				ver_1 = makeRectangle(j.dimensions[0], j.dimensions[2], j.angle, (j.pos[0], j.pos[2]))
			
				for f in fixed:
					ver_f =  makeRectangle(f.dimensions[0], f.dimensions[2], f.angle, (f.pos[0], f.pos[2]))
					if overlap(ver_0, ver_f):
						i.angle = i.angle/2.0
						i = move_idle_right(i, right)
					if(overlap(ver_1, ver_f)):
						j.angle = j.angle/2.0
						j = move_idle_left(j, left) 
				
			idle[0] = i
			idle[1] = j


	if (len(collide) > 1):
		
		#for o1, o2 in zip(coll, coll[1:]):
		for i in range(len(coll)):
						

			ind2 = (i + 1)% len(coll)

			o1 = coll[i]
			o2 = coll[ind2]
	
			arr  = [o1,o2]
			arr.sort(key = lambda x: x.pos[0])

			left = arr[0]
			right = arr[-1]			

			ind_1 = coll.index(o1)
			ind_2 = coll.index(o2)
			print "comparing %1d"%ind_1
			print "and %1d"%ind_2
			o1_ver = makeRectangle(o1.dimensions[0], o1.dimensions[2], o1.angle, (o1.pos[0], o1.pos[2]))
			o2_ver = makeRectangle(o2.dimensions[0], o2.dimensions[2], o2.angle, (o2.pos[0], o2.pos[2]))

			if(overlap(o1_ver, o2_ver)):
				
				print "fixing %1d"%ind_1
				print "and %1d"%ind_2
				# decide which direction should move, and we move 
				#new1, new2 = fix_movables(o1, o2, o1_ver, o2_ver, fixed)
				(new1, new2) = fix_movables((left, right), fixed)

				coll[ind_1] = new1
				coll[ind_2] = new2

	# check if the movable overlap with fixed object
	
	if (len(collide) > 0):
		for o in coll :
		
			
			ver = makeRectangle(o.dimensions[0], o.dimensions[2], o.angle, (o.pos[0], o.pos[2]))
			
			for fix in reversed( fixed ):

				
				f_ver =  makeRectangle(fix.dimensions[0], fix.dimensions[2], fix.angle, (fix.pos[0], fix.pos[2]))

				if(overlap(ver, f_ver)):
					print ("fixed index ", fixed.index(fix))
					ind =  coll.index(o)
					p1 = (o.pos[0], o.pos[2])
					p2 = (fix.pos[0], fix.pos[2])
										
					#TODO
					#d = get_point_line_dist(p1, p2, fixed.angle)
					#d_1 = o.dimensions[0]/2.0 - (d/math.sin(o.angle))
					#shift_d  = d_1 + (fixed.dimensions[2]/2.0)*((1/math.sin(o.angle)) + (1/math.tanh(o.angle)))
					#shift_d = get_shift_point_dist(ver, f_ver)
					shift, up = get_shift_dist(o, fix, fixed)
					#print ("shift_d" , shift_d)					
					#shift = shift_d * math.cos(o.angle)
					print ("shift" , shift)
					#up = shift_d * math.sin(o.angle)
					print ( "up" , up)
					#shift = (o.dimensions[0] - d ) * math.sin(o.angle)
					#if shift_d > 0.15 :
					#	shift = 0
					#	up = 0
					#if  fix.pos[0] < 0.25:
					#	print "move down"
					#	o.pos[0] -= abs(shift)
					#else : o.pos[0] += abs(shift)
					
					
					#up = (o.dimensions[0] - d  )  * math.cos(o.angle)
					if o.pos[0] < 0.25:
						o.pos[0] -= abs(shift)
					else:
						o.pos[0] += abs(shift)
					o.pos[2] += up
	
			
			if(len(idle) > 0):
				for i in idle :
				
					ver_i =  makeRectangle(i.dimensions[0], i.dimensions[2], i.angle, (i.pos[0], i.pos[2]))

					if (overlap(ver, ver_i)):
						print "hit idle"
						ind = idle.index(i)
						shift_d = get_shift_point_dist(ver, ver_i)					
						shift = shift_d * math.cos(o.angle)
						up = shift_d * math.sin(o.angle)	
						#left side
						if(i.angle > 0):
							#i.angle = i.angle/2.0 
							#i = move_idle_left(i, left)
							# put it somewhere else
							i.pos[0] = 0.6
							i.pos[2] = 0.5
						# right side
						elif(i.angle < 0):
							i.pos[0] += shift
							i.pos[2] -+ up
						idle[ind] = i



	return coll+idle+fixed



# distance from point p0 to line formed by p1, p2
def distance(p0, p1, p2): # p0 is the point
    x0, y0 = p0
    x1, y1 = p1
    x2, y2 = p2
    nom = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
    denom = math.sqrt(((y2 - y1)**2 + (x2 - x1) ** 2))
    result = nom / denom
    return result


def move_idle_right(i, right):
	i.angle = -abs(i.angle)

	ver = makeRectangle(i.dimensions[0], i.dimensions[2], i.angle, (i.pos[0], i.pos[2]))
	copy = ver
	copy.sort(key = lambda x: x[0])
	ver_right = makeRectangle(right.dimensions[0], right.dimensions[2], right.angle, (right.pos[0], right.pos[2]))
	copy_right = ver_right
	copy_right.sort(key = lambda x: x[0])

	shift = copy_right[-1][0] - copy[0][0]
			
	i.pos[0] += shift
	
	copy.sort(key = lambda x: x[1])
	copy_right.sort(key = lambda x: x[1])
	up = copy_right[-1][1] - ver[1][1]
			
	i.pos[2] += up
	
	return i 
			
def move_idle_left(i, left):
	
	#if(idle[1].angle > 0 ):
	i.angle = abs(i.angle)
	ver_l = makeRectangle(i.dimensions[0], i.dimensions[2], i.angle, (i.pos[0], i.pos[2]))

	copy_l = ver_l
	copy_l.sort(key = lambda x: x[0])
	ver_left = makeRectangle(left.dimensions[0], left.dimensions[2], left.angle, (left.pos[0], left.pos[2]))
	copy_left = ver_left
	copy_left.sort(key = lambda x: x[0])

	shift = copy_left [0][0] - copy_l[-2][0]

	copy_l.sort(key = lambda x: x[1])
	copy_left.sort(key = lambda x: x[1])
			
	i.pos[0] += shift
	up = copy_left[-1][1] - copy_l[0][1]
			
	i.pos[2] += up

	return  i
			 			
# FIXME sth wrong here
#def fix_movables(o1, o2, o1_ver, o2_ver, fixed):
def fix_movables(o_list, fixed):
	
	o1 = o_list[0]
	o2 = o_list[-1]
	new = []
	print o1.pos[0], o2.pos[0]
	x_1 = o1.pos[0]
	x_2 =  o2.pos[0]
	dim_x_1 = o1.dimensions[0] 
	dim_x_2 = o2.dimensions[0]

	right = fixed[-2]
	
	left = fixed[-3]

	if (right.pos[0] < left.pos[0]):
		tmp = right
		right = left
		left = tmp

	ver_right = makeRectangle(right.dimensions[0], right.dimensions[2], right.angle, (right.pos[0], right.pos[2]))
	ver_left = makeRectangle(left.dimensions[0], left.dimensions[2], left.angle, (left.pos[0], left.pos[2]))
	ver_left.sort(key = lambda x: x[0])
	left_bound = ver_left[0][0]
	
	# move it to the left
	#if (x_1 < dim_x_1 ):
		#print "case 1 : x_1 < dim_x_1"
		#if(left_x_2 > 0):
		# otherwise the ball may have not enough energy to roll
		#print o2.angle
		#if (o2.angle < 0):
			#o2.angle = abs(o2.angle)
	p1 = (o1.pos[0], o1.pos[2])
		#angle1 = o1.angle
	p2 = (o2.pos[0], o2.pos[2])
	d = get_point_line_dist(p1, p2, o2.angle)

	o1_ver = makeRectangle(o1.dimensions[0], o1.dimensions[2], o1.angle, (o1.pos[0], o1.pos[2]))
	o2_2_ver = makeRectangle(o2.dimensions[0], o2.dimensions[2], o2.angle, (o2.pos[0], o2.pos[2]))
	o2_2_ver.sort(key = lambda x: x[0])
	left_x_2 = o2_2_ver[0][0]
		
	# FIXME wrong
	if(x_1 > -(dim_x_1)/4.0) and (x_2 < dim_x_2):
		print "shift the left one"
		#print d

		#d_1 = o1.dimensions[0]/2.0 - (d/math.sin(o1.angle))
		#d_1 = o1.dimensions[0]/2.0
		#shift_d  = d_1 + (o2.dimensions[2]/2.0)*((1/math.sin(o1.angle)) + (1/math.tanh(o1.angle)))
		#shift_d = d_1 + (o2.dimensions[2]/2.0)*((1/math.sin(o1.angle)) + math.tanh(o1.angle)) 
		shift_d = get_shift_point_dist(o1_ver, o2_2_ver)

		shift = shift_d * math.cos(o1.angle)
		#shift = (o1.dimensions[0] - d * math.cos(o1.angle) + (o2.dimensions[2]/2.0)*math.sin(o1.angle) ) * math.cos(o1.angle)
		print "shift"
		print shift
		o1.pos[0] -= abs(shift)
		#up = (o1.dimensions[0] - d )  * math.sin(o1.angle)
		up = shift_d * math.sin(o1.angle)
		
		if up < 0.001:
			print "change up"
			'''
			o2_2_ver.sort(key = lambda x: x[1])
			o1_ver.sort(key = lambda x: x[1])
			low = o2_2_ver[-1][1]
			high = o1_ver[0][1]
			up = high - low	
			'''
			up += 0.0001
			
		print "up"
		print up
	
		o1.pos[2] += abs(up)

		'''
		ver = makeRectangle(o1.dimensions[0], o1.dimensions[2], o1.angle, (o1.pos[0], o1.pos[2]))
		if overlap(ver, o2_2_ver):
			print "change up"
			o2_2_ver.sort(key = lambda x: x[1])
			o1_ver.sort(key = lambda x: x[1])
			low = o2_2_ver[-1][1]
			high = o1_ver[0][1]
			up = high - low	
			print "up"
			print up
			
			o1.pos[2] += abs(up)
		'''	
	else:
		print "shift the right one"
			#print shift
		

		#d_1 = o1.dimensions[0]/2.0 - (d/math.sin(o1.angle))
		shift_d = get_shift_point_dist(o1_ver, o2_2_ver)
		#shift_d  = d_1 + (o2.dimensions[2]/2.0)*((1/math.sin(o1.angle)) + (1/math.tanh(o1.angle)))
		shift = shift_d * math.cos(o1.angle)
		#shift = (o2.dimensions[0] - d) * math.cos(o1.angle)
		o2.pos[0] += abs(shift)
			# it should, otherwise won't collide if o1 is near 0 in x axis
			#if(shift > 0):
		#	o2.pos[0] += shift
		up = shift_d * math.sin(o1.angle)
		#up = (o2.dimensions[0] - d )  * math.sin(o1.angle)
		if up < 0.001:
			o2_2_ver.sort(key = lambda x: x[1])
			o1_ver.sort(key = lambda x: x[1])
			low = o2_2_ver[-1][1]
			high = o1_ver[0][1]
			up = high - low	
		o2.pos[2] -= abs(up)
		
			

	#o2_2_ver = makeRectangle(o2.dimensions[0], o2.dimensions[2], o2.angle, (o2.pos[0], o2.pos[2]))
	
	

	#ver_1 = makeRectangle(o1.dimensions[0], o1.dimensions[2], o1.angle, (o1.pos[0], o1.pos[2]))
	#ver_2 =  makeRectangle(o2.dimensions[0], o2.dimensions[2], o2.angle, (o2.pos[0], o2.pos[2]))
	#print overlap(ver_1, ver_2)
	new.append(o1)
	new.append(o2)
	return new

def get_overlap_count(obst):
	
	for o in obst:
		o.dimensions = np.array(o.dimensions, dtype=np.float32)
		o.pos = np.array(o.pos, dtype=np.float32)
		o.angle = float(o.angle)
	
	c = 0
	movable = []
	fixed = []
	#print len(obst)	
	if len(obst) == 7 :
		# one fixed obstacle
		movable = obst[:3]
		
		fixed = obst[-4:]
		

	#print movable
  	#print fixed

	# from left to right in x direction
	movable.sort(key = lambda x: x.pos[0])
	#fixed.sort(key = lambda x: x.pos[0])
	#print (" do sth")
	
	
	#global draw
	for o1, o2 in zip(movable, movable[1:]):

		
		o2_ver = makeRectangle(o2.dimensions[0], o2.dimensions[2], o2.angle, (o2.pos[0], o2.pos[2]))		

		o1_ver = makeRectangle(o1.dimensions[0], o1.dimensions[2], o1.angle, (o1.pos[0], o1.pos[2]))

		#print ("movable %1d, "%movable.index(o1) + "movable %1d"%movable.index(o2))
		if(overlap(o1_ver, o2_ver)):
		
			print ("hit movable %1d, "%movable.index(o1) + "movable %1d"%movable.index(o2))
			
			c += 1

	for i in range(len(fixed)):
		
		o = fixed[i]
		v = makeRectangle(o.dimensions[0],o.dimensions[2], o.angle, (o.pos[0], o.pos[2]))
		
		for m in movable:
			#if ( movable.index(m) == 2 and i == 0):
				#print ("attention error ")				
			
			ver_m =  makeRectangle(m.dimensions[0],m.dimensions[2], m.angle, (m.pos[0], m.pos[2]))	
			#print (" movable %1d, "%movable.index(o1) + "movable %1d"%movable.index(o2))		
			if(overlap(v, ver_m)):
				print ("hit movable %1d"%movable.index(m) + " fixed %1d "%i)
				c += 1

	return c



# it also works for convex polygons
def overlap(ver1, ver2):
	
	# each vertices set are in top-right, right-under, left-under, top-left oder 	

	rects = [ver1, ver2]
	
	global draw
	#projected = 0.0

	for r in rects :
		
		for i in range(len(r)):
			min_1 = None
			min_2 = None
			max_1 = None
			max_2 = None			

			ind2 = (i + 1)% len(r)
			p1 = r[i]
			p2 = r[ind2]

			
			normal = (p2[1] - p1[1] , p1[0] - p2[0])
			#draw.line(normal + )
			#print normal
			#print ver1
			#print ver2
			for v in ver1:		
				projected = normal[0] * v[0] + normal[1] * v[1]

				if ( not min_1 or projected < min_1 ):
					min_1 = projected
				if (not max_1 or projected > max_1):
					max_1 = projected

			for v in ver2:		
				projected2 = normal[0] * v[0] + normal[1] * v[1]

				if (not min_2 or projected2 < min_2):
					min_2 = projected2
				if (not max_2 or projected2 > max_2):
					max_2 = projected2

			
			#print max_1
			#print min_1
			#print max_2
			#print min_2
			#print ("   " )
			
			if (max_1 < min_2 or max_2 < min_1):
				return False

	return True

# distance to shift along first obstacle's length
def get_shift_point_dist(ver1, ver2):
	print ver1
	print ver2
	
	ver1.sort(key = lambda x: x[1])
	under = ver1[0]
	
	len_l = ver1[1]
	
	tmp = ver1[2]

	# find the under length edge of the first obstacle
	d1 = math.sqrt((under[0] - len_l[0])**2 + (under[1] - len_l[1])**2)
	d2 = math.sqrt((under[0] - tmp[0])**2 + (under[1] - tmp[1])**2)
	
	if d2 > d1 :
		len_l = ver1[2]
	
	ver2.sort(key = lambda x: x[1])
	
	p = ver2[-1]

	left_up = p


	can_1 = ver2[-2]
	can_2 = ver2[-3]
	d1 = math.sqrt((p[0]- can_1[0])**2 + (p[1]- can_1[1])**2 )
	d2 = math.sqrt((p[0]- can_2[0])**2 + (p[1]- can_2[1])**2 )
	
	# find the upper length edge of second obstacle
	if(d2 > d1):
		left_up = can_2
	else:
		left_up = can_1

	print ("under ", under )
	print ("len_l ", len_l)
	print ("p ", p )
	print ("left_up ", left_up )
	intersect = get_intersect(under, len_l, p, left_up)

	print intersect 

	print ("diff 1 : ", under[0] - intersect[0])
	print ('diff 2 : ', under[1] - intersect[1])
	d = math.sqrt((under[0] - intersect[0])**2 + (under[1] - intersect[1])**2)
	print ("d first calculated ", d)

	left = min(p[0], left_up[0])
	right = max(p[0], left_up[0])
	up = max(p[1], left_up[1])
	down = min(p[1], left_up[1])
	# if the intersect point is not on the line segment
	if (intersect[0] < left ) or (intersect[0] > right) or (intersect[1] < down) or (intersect[1] > up):
		print "change"
		if p[0] < under[0]:
		
			d = math.sqrt((p[0] - under[0])**2 + (p[1] - under[1])**2)

		else:
			d = math.sqrt((left_up[0] - under[0])**2 + (left_up[1] - under[1])**2)

	print ("d before return ", d)
	return d

# distance for movable obstacle to shift while colliding with a fixed object
def get_shift_dist(o, fix, l_fixed):

	ver = makeRectangle(o.dimensions[0], o.dimensions[2], o.angle, (o.pos[0], o.pos[2]))

	f_ver = makeRectangle(fix.dimensions[0], fix.dimensions[2], fix.angle, (fix.pos[0], fix.pos[2]))

	#if (fix.pos[0] > 0.25):
	ver.sort(key = lambda x: x[1])
	under = ver[0]
	
	len_l = ver[1]
	
	tmp = ver[2]

	# find the under length edge of the first obstacle
	d1 = math.sqrt((under[0] - len_l[0])**2 + (under[1] - len_l[1])**2)
	d2 = math.sqrt((under[0] - tmp[0])**2 + (under[1] - tmp[1])**2)
	
	if d2 > d1 :
		len_l = ver[2]
	
	#f_ver.sort(key = lambda x: x[0])
	f_ver.sort(key = lambda x: x[1])
	
	p = f_ver[-1]

	left_up = f_ver[-2]


	
	
	intersect = get_intersect(under, len_l, p, left_up)
	d1 = math.sqrt((under[0] - intersect[0])**2 + (under[1] - intersect[1])**2)
	print ("d1 " , d1)

	print ('intersect point : ', intersect) 
	#if o.pos[0] < fix.pos[0]:
	d2 = math.sqrt((len_l[0] - intersect[0])**2 + (len_l[1] - intersect[1])**2)
	print ("d2 " , d2)
	d = min(d1, d2)
	print ("d ", d)
	left = min(p[0], left_up[0])
	#print ("left ", left)
	right = max(p[0], left_up[0])
	#print ("right " , right)
	up = max(p[1], left_up[1])
	#print ("up ", up)	
	down = min(p[1], left_up[1])
	
		
		#print ('down ', down)
		# if the intersect point is not on the line segment
	#if (intersect[0] < left ) or (intersect[0] > right) :
	#	print "change"
	'''
		if p[0] < under[0]:
		
			d1 = math.sqrt((p[0] - under[0])**2 + (p[1] - under[1])**2)
			d2 = math.sqrt((p[0] - len_l[0])**2 + (p[1] - len_l[1])**2)
			d = min(d1, d2)

		else:
			d1 = math.sqrt((left_up[0] - under[0])**2 + (left_up[1] - under[1])**2)
			d2 = math.sqrt((left_up[0] - len_l[0])**2 + (left_up[1] - len_l[1])**2)
			d = min(d1 , d2)
	'''
	print ("d before return ", d)
	delta_x = d* math.cos(o.angle)
	delta_z = d*math.sin(o.angle)
	shifted_x = o.pos[0] + d* math.cos(o.angle)
	shifted_z = d*math.sin(o.angle) + o.pos[2]

	shifted = makeRectangle(o.dimensions[0], o.dimensions[2], o.angle, (shifted_x, shifted_z))

	if overlap(shifted , f_ver):
		print "not fixed yet"
		d = max(d1, d2)
		delta_x = d* math.cos(o.angle)
		delta_z = d*math.sin(o.angle)
	#f_ver = makeRectangle(fix.dimensions[0], fix.dimensions[2], fix.angle, (fix.pos[0], fix.pos[2]))

		
	return delta_x, delta_z
	

	


# get intersect point of two lines
def get_intersect(a1, a2, b1, b2):
    """ 
    Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
    a1: [x, y] a point on the first line
    a2: [x, y] another point on the first line
    b1: [x, y] a point on the second line
    b2: [x, y] another point on the second line
    """
    s = np.vstack([a1,a2,b1,b2])        # s for stacked
    h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
    l1 = np.cross(h[0], h[1])           # get first line
    l2 = np.cross(h[2], h[3])           # get second line
    x, y, z = np.cross(l1, l2)          # point of intersection
    if z == 0:                          # lines are parallel
        return (float('inf'), float('inf'))
    return (x/z, y/z)



# get the distance from p1 to the line formed by p2 and the angle			
def get_point_line_dist(p1, p2, angle2):

	slope = math.tanh(angle2)
	
	b = p2[1] - slope*p2[0]
	

	# compute point projected on the line by p2
	denom = slope + 1/slope

	numerator = p1[1] + (1/slope)*p1[0] + slope*p2[0] - p2[1]

	x = numerator/denom

	y = slope * x + b

	d = math.sqrt((p1[0] - x)**2 + (p1[1] - y)**2)

	return d
	

def makeRectangle(l, w, theta, offset=(0,0)):


	#print theta
	c, s = math.cos(theta), math.sin(theta)
	#s, c = math.cos(theta), math.sin(theta)
	#print c,s
	#c,s = math.sin(theta), math.cos(theta)
	rectCoords = [(l/2.0, w/2.0), (l/2.0, -w/2.0), (-l/2.0, -w/2.0), (-l/2.0, w/2.0)]
	#c, s = math.cos(-theta), math.sin(-theta)
	#rectCoords = [(l/2.0, w/2.0), (l/2.0, -w/2.0), (-l/2.0, -w/2.0), (-l/2.0, w/2.0)]
   	#if theta > 0 :
		#rectCoords = [(l/2.0, w/2.0), (l/2.0, -w/2.0), (-l/2.0, w/2.0), (-l/2.0, -w/2.0)]
	
	# top-right, right-under, left-under, top-left
	return [(c*x + s*y+offset[0], - s*x+c*y+offset[1]) for (x,y) in rectCoords]

'''
obst = get_valid_output(get_init_setup())
#print get_overlap_count(get_init_setup())

f1 = open('./tests/checked_setup.txt', 'w')

for o in obst:
	f1.write("o ")
	f1.write(str(o.dimensions[0]) + ' ' + str(DIMENSION_Y) + ' ' + str(o.dimensions[2]) + ' ')
                	#print o.pos[0][0]
	f1.write(str(float(o.pos[0])) + ' 0.0 ' + str(float(o.pos[2])) + ' ' + str(float(o.angle))+'\n' )

f1.close()


#print get_overlap_count(get_init_setup())
'''

