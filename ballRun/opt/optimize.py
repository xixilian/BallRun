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
import cma
import math
import check



BALL_DIAMETER = 0.017270193333
DIMENSION_X = 0.287351820006
DIMENSION_Y = 0.01
DIMENSION_Z = 0.0425280693609

start = 0.0
end = 0.0
#hit = []

#hit_cost = []

hit_counter = 0
best_obst = []
best_score = 100

t = time.clock()

sum_time = []

loopcounter = 0

bounds = [[0, 0.45], [0, 0.45], [-1, 1],
          [0, 0.45], [0, 0.45], [-1, 1],
          [0, 0.45], [0, 0.45], [-1, 1]]

get_params = None

cost_function = None

class Goal():

	def __init__(self):
		self.obstacles = []
		self.pos_x = 0.0
		self.pos_z = 0.0
		self.total_height = 0.0
		self.height = 0.0
		self.left_bound = 0.0
		self.right_bound = 0.0
		self.len = 0.0 
		
goal = Goal()

def f(x, *args):
  	obst = get_params(x)
  	result = run_prog_process(obst)
  	cost = cost_function(result, obst)

	return cost

def get_params_part_3(x):

  obst = []
 
  for i in range(0,3):
	
  	o = Obstacle()
  	o.dimensions.append(DIMENSION_X)
  	o.dimensions.append(DIMENSION_Y)
  	o.dimensions.append(DIMENSION_Z)
  	o.pos.append(x[i*3])
  	o.pos.append(0.0)
  	o.pos.append(x[i*3+1])
  	o.angle = x[i*3+2]

  	obst.append(o)

  obst.sort(key = lambda x: x.pos[0])
  return obst 


	  
def get_params_part_4(x):

	obst = []
  #print(str(x))
	for i in range(0,3):
	#print i
  		o = Obstacle()
  		o.dimensions.append(DIMENSION_X)
  		o.dimensions.append(DIMENSION_Y)
  		o.dimensions.append(DIMENSION_Z)
  		o.pos.append(x[i*3])
  		o.pos.append(0.0)
  		o.pos.append(x[i*3+1])
  		o.angle = x[i*3+2]

  		obst.append(o)

	# fixed one
	o = Obstacle()
	o.dimensions.append(DIMENSION_X - 0.05)
  	o.dimensions.append(DIMENSION_Y)
  	o.dimensions.append(DIMENSION_Z)
 	o.pos.append(0.4)
 	o.pos.append(0.0)
  	o.pos.append(0.25)
  	o.angle = 0.0

  	obst.append(o)
	
  	return obst 



def cost_part_4(r, obst):
	c = cost_part_3(r, obst )
	return c

def cost_part_3(r, obst):

	global goal, hit, hit_cost, best_obst, best_score, loopcounter, t, sum_time, start, end
	
	#t0 = time.time()
	#sum_time.append(time.time())
	f2 = open('./results/rstring.txt', 'r')
	l = f2.readlines()
	re = l[0].strip()
	f2.close()

	penalty = 1
	
	if re[0] == 's':
	
		penalty = 0
			
	if (len(r) == 4):	
	  	x, y = float(r[0]), float(r[1])
		v = float(r[-2])
		z_pos = float(r[-1])
	  	goal_pos_x = float(goal.obstacles[-1].pos[0])
	  	goal_pos_z = float(goal.obstacles[-1].pos[2])
		goal_height = float(goal.total_height)
		
		c = (x - goal_pos_x) ** 2  + (-0.05 - v)**2 + (z_pos - goal_height)**2 + (obst[-1].pos[0] - goal_pos_x - DIMENSION_X/2.0)*0.3
		
  		return c
	elif (len(r) == 11):
		x, y = float(r[0]), float(r[1])
		v_x_up = float(r[2])
		v_z_up = float(r[3])
		z_pos_up = float(r[4])
		x_pos_up = float(r[5])

		v_x_down = float(r[-5])
		v_z_down = float(r[-4])
		z_pos_down = float(r[-3])
		x_pos_down = float(r[-2])
		bounce_c = int(r[-1])

	  	goal_pos_x = float(goal.obstacles[-1].pos[0])
	  	goal_pos_z = float(goal.obstacles[-1].pos[2])
		goal_height = float(goal.total_height)
		
	
		all_obst = obst
		if (len(obst) < 6):
			all_obst = obst + goal.obstacles
		overlap_c = check.get_overlap_count(all_obst)

		#c = 5*((x - goal_pos_x + penalty) ** 2)  +  (v_x_up)**2 +(v_x_down)**2+ 5*((x_pos_down - goal_pos_x)**2) +5*((x_pos_up - goal_pos_x)**2) + (y - goal_pos_z - round((BALL_DIAMETER/2),5) - goal.height/2 ) ** 2 + ((obst[-1].pos[0] - math.cos(obst[-1].angle)*(DIMENSION_X/2.0)) - goal_pos_x  - goal.len/2.0)**2*0.05 +((obst[-1].pos[2] - math.sin(obst[-1].angle)*((DIMENSION_X/2.0))) - goal.total_height - DIMENSION_Z/2.0 )**2*0.05 + overlap_c * 0.08
		c = 5*((x - goal_pos_x + penalty) ** 2)  +  (v_x_up)**2 +(v_x_down)**2+ 5*((x_pos_down - goal_pos_x)**2) +5*((x_pos_up - goal_pos_x)**2) + (y - goal_pos_z - round((BALL_DIAMETER/2),5) - goal.height/2 ) ** 2  + overlap_c * 50
		
		# the score function		
		c2 = 5* v_x_up **2 + 5 *v_z_up**2 + bounce_c + 10*((x_pos_up - goal_pos_x +1)**2)
		#f3.write(str(c) + '\n')
		#f3.close()
		#print c
		'''
		if re[0] == 's' and overlap_c == 0:
			hit_cost.append(c)
			hit.append(obst)
			if c2 < best_score :
				best_score = c2
				best_obst = all_obst
            if (len(hit) > 10):
                path = "./tests/best_setup.txt"
                write_setup(best_obst, path)
                print("run time :", sum(sum_time))
                sys.exit(0)
		'''

		# test cases, for stat
		path = './dyn_results/%04d.txt'%loopcounter
		write_setup(all_obst, path)
		
		'''
		if re[0] == 's' and overlap_c == 0:
			#sum_time.append(time.time())
			end = time.time()
			print("run time :", end - start)
			print c2
			sys.exit(0)
		
		'''
		if re[0] == 's' and overlap_c == 0:
			hit_cost.append(c)
			hit.append(obst)
			if c2 < best_score :
				best_score = c2
				best_obst = all_obst
			print(" hit list len :", len(hit))
            	if (len(hit) > 10):
                	path = "./tests/best_setup.txt"
                	write_setup(best_obst, path)
			end = time.time()
			print(" hit list len :", len(hit))
                	print("run time :", end - start)
			print("best score : ", best_score)
                	sys.exit(0)
		
		loopcounter += 1
		return c




def cost_part_3_v1(positions):
	global goal
	f2 = open('./results/rstring.txt', 'r')
	l = f2.readlines()
	r = l[0].strip()
	f2.close()
	if r[0] == 's':
		return 0.0		
		
  	x, y = round(float(positions[-2]), 4), round(float(positions[-1]), 4)
  	goal_pos_x = round(float(goal[-1].pos[0]), 4)
  	goal_pos_z = round(float(goal[-1].pos[2]),4)
	c = (x - goal_pos_x) ** 2 + (y - goal_pos_z - round((BALL_DIAMETER/2),5)) ** 2
	#print c
  	return c

def get_bounds_part_4():
  return [[0, 0.45], [0, 0.45], [-1, 1],
          [0, 0.45], [0, 0.45], [-1, 1],
          [0, 0.45], [0, 0.45], [-1, 1]]


def get_bounds_part_3():
  return [[0, 0.45], [0, 0.45], [-1, 1],
          [0, 0.45], [0, 0.45], [-1, 1],
          [0, 0.45], [0, 0.45], [-1, 1]]
          
def method_differential_evolution():
  result = differential_evolution(f, bounds,popsize = 5, maxiter= 20, disp= False, atol =1)
  return result.fun, result.x

def get_random_x0():
  randn = np.random.random_sample(len(bounds))
  return np.array([x[0] + a*(x[1]-x[0]) for x,a in zip(bounds, randn) ])


def get_x0():
	import shutil
	# change the file directory here
	shutil.copy2('./params/dyn_setup.txt', './params/setup.txt')
	obst = get_init_setup()
	x = []
	for o in obst[:3]:
		x.append(float(o.pos[0]))
		x.append(float(o.pos[2]))
		x.append(float(o.angle))

	return x 



def _method_cg_eps(eps):
  result = scipy.optimize.minimize(f, get_random_x0(), method='CG', options={'disp' : False, 'eps' : eps , 'maxiter' : 20})
  #result = scipy.optimize.minimize(f, get_x0(), method='CG', options={'disp' : False, 'eps' : eps })
  return result.fun, result.x

def method_cg_eps_low():
  return _method_cg_eps(0.01)

def method_cg_eps_high():
  return _method_cg_eps(0.1)

def method_slsqp_high():
  	result = scipy.optimize.minimize(f, get_random_x0(), method='SLSQP', options={'disp' : False, 'eps' : 1 , 'maxiter' : 20})
	#result = scipy.optimize.minimize(f, get_x0(), method='SLSQP', bounds = bounds , options={'disp' : False, 'eps' : 0.5 , 'maxiter' : 20})
  	return result.fun, result.x

def method_slsqp_low():
  
	result = scipy.optimize.minimize(f, get_random_x0(), method='SLSQP', options={'disp' : False, 'eps' : 1e-8 , 'maxiter' : 20})
	#result = scipy.optimize.minimize(f, get_x0(), method='SLSQP', bounds = bounds , options={'disp' : False, 'eps' : 0.1 , 'maxiter' : 5})
  	return result.fun, result.x

def method_bfgs_low():
        result = scipy.optimize.minimize(f, get_random_x0(), method='BFGS', options={'disp' : False, 'eps' : 1e-8, 'maxiter':args.opt_iters })
	#result = scipy.optimize.minimize(f, get_x0(), method='BFGS', options={'disp' : False, 'eps' : 1e-8, 'maxiter':args.opt_iters })
  	return result.fun, result.x

def method_bfgs_high():
  	result = scipy.optimize.minimize(f, get_random_x0(), method='BFGS', options={'disp' : False, 'eps' : 1, 'maxiter':args.opt_iters })
	#result = scipy.optimize.minimize(f, get_x0(), method='BFGS', options={'disp' : False, 'eps' : 0.1, 'maxiter':args.opt_iters })
  	return result.fun, result.x


def method_cma():
  	
	#x0 = get_x0()
	x0 = get_random_x0()
  	es = cma.fmin(f, x0, 0.1, options={
                              'popsize': 80,
                              #'tolfun': 1e-2,
                              'maxfevals': 5000,
                              #'tolx' : 1e-3,
                              'bounds': [[x[0] for x in bounds], [x[1] for x in bounds ] ]},
                              restarts=0)
  	return es[1], es[0]

@multimethod(list)
def write_setup(obstacles):

	global goal
	f1 = open("./params/dyn_setup.txt", 'w')

	for o in obstacles:
	    f1.write("o ")
	    f1.write(str(o.dimensions[0]) + ' ' + str(DIMENSION_Y) + ' ' + str(o.dimensions[2]) + ' ')    
	    f1.write(str(float(o.pos[0])) + ' 0.0 ' + str(float(o.pos[2])) + ' ' + str(float(o.angle))+'\n' )

	if(len(obstacles) < 7):
		for o in goal.obstacles:
			f1.write("o ")
			f1.write(str(o.dimensions[0]) + ' ' + str(o.dimensions[1]) + ' ' + str(o.dimensions[2]) + ' ' )
			f1.write(str(o.pos[0]) + ' ' + str(o.pos[1]) + ' ' + str(o.pos[2]) + ' ')
			f1.write(str(o.angle) + '\n')

	f1.close()

@multimethod(list, str)
def write_setup(obstacles, path):
	global goal


	f1 = open(path, 'w')

	for o in obstacles:
	    f1.write("o ")
	    f1.write(str(o.dimensions[0]) + ' ' + str(DIMENSION_Y) + ' ' + str(o.dimensions[2]) + ' ')
                	#print o.pos[0][0]
	    f1.write(str(float(o.pos[0])) + ' 0.0 ' + str(float(o.pos[2])) + ' ' + str(float(o.angle))+'\n' )

	if(len(obstacles) < 7):
		for o in goal.obstacles:
			f1.write("o ")
			f1.write(str(o.dimensions[0]) + ' ' + str(o.dimensions[1]) + ' ' + str(o.dimensions[2]) + ' ' )
			f1.write(str(o.pos[0]) + ' ' + str(o.pos[1]) + ' ' + str(o.pos[2]) + ' ')
			f1.write(str(o.angle)+ '\n')

	f1.close()

def get_init_setup():

	obst = []
  	
	f1 = open("./params/setup.txt", 'r')
	
	lines = f1.readlines()
	for line in lines:
	
		cont = line.split(' ')
		
		if len(cont) == 8:
  			o = Obstacle()
  			o.dimensions.append(cont[1])
  			o.dimensions.append(cont[2])
  			o.dimensions.append(cont[3])
  			o.pos.append(cont[-4])
  			o.pos.append(cont[-3])
  			o.pos.append(cont[-2])
 			o.angle = cont[-1]

  			obst.append(o)

	obst[:3]
        #	del obst[-3:]
	#elif(len(obst) == 7):
	#	del obst[-4:]
	print len(obst)
	f1.close()
  	return obst 


def read_goal_setup():

		# for the reset
		goal = Goal()

		# the goal in this setup file is shorter
		f = open("./params/goal_setup_short_backup.txt", 'r')
		# normal setup	
		#f = open("./params/goal_setup.txt", 'r')

		
		for line in f :
			o = Obstacle()
			contents = line.split(' ')
			
			
			if len(contents) == 7:
				o.dimensions.append(contents[0])
				o.dimensions.append(contents[1])
				o.dimensions.append(contents[2])
				o.pos.append(contents[3])
				o.pos.append(contents[4])
				o.pos.append(contents[5])
				o.angle = contents[6]
				goal.obstacles.append(o)

			# for ode the line needs to begin with 'o'
			if len(contents) == 8:
				o.dimensions.append(contents[1])
				o.dimensions.append(contents[2])
				o.dimensions.append(contents[3])
				o.pos.append(contents[4])
				o.pos.append(contents[5])
				o.pos.append(contents[6])
				o.angle = contents[7]
				goal.obstacles.append(o)


		f.close()


		goal.obstacles.sort(key = lambda x: x.dimensions[0])

		# the sort is in ascending order
		
		mid = goal.obstacles[-1]
		
		
		goal_len = float(mid.dimensions[0])
		goal.len = goal_len
		goal_height = float(mid.dimensions[2])
		goal.pos_x = float(mid.pos[0])
		goal.pos_z = float(mid.pos[2])
		goal.left_bound = goal.pos_x - goal.len/2.0
		goal.right_bound = goal.left_bound + goal_len
		small_piece = goal.obstacles[0]
		goal.total_height = float(small_piece.dimensions[2]) + goal_height
		goal_x_offset = float(small_piece.dimensions[0]) /2.0
		#goal_y_offset = round(float(small_piece.dimensions[2]) /2.0,4) + self.goal_height
		goal.height = goal_height

		# reset goal position
		reset = np.random.randint(0,2)
	
		if (reset == 0):
			goal.pos_x = np.random.uniform(0.3, 0.55) 
			goal.obstacles[-1].pos[0] = goal.pos_x
			left_x = goal.pos_x - goal_len/2.0 + goal_x_offset
			goal.obstacles[0].pos[0] = left_x
			right_x =  goal.pos_x + goal_len/2.0 - goal_x_offset
			goal.obstacles[1].pos[0] = right_x

		return goal
			
		
def run_prog_process(obs):
	
	global goal
	all_obs = obs
	if (len(obs) < 6):
		all_obs += goal.obstacles
	n_obst = check.get_valid_output(all_obs)
  	write_setup(n_obst)
  	subprocess.call("./dynamics", shell= False)
  	f = open('./results/result.txt', 'r')
	l = f.readlines()
	result = l[0].split(" ")
	f.close()
	return result
	


class Obstacle():

	def __init__(self):
		self.dimensions = []
		self.pos = []
		self.angle = 0.0


def run(f):
	costs = []
	times = []
	xs = []

	index = []

	global hit, hit_cost, goal, sum_time, start

	#sum_time.append(time.time())
	#if os.path.exists("./tests/break_test.txt"):
	#	os.remove("./tests/break_test.txt")
	#test = open("./tests/break_test.txt", 'a')

	for i in range(10):
		times.append(time.clock())
	
		#test.write(str(i) + "\n")
		#test.write(str(len(hit)) + '\n')
		
		t = time.time()
		#if sum(times) > 60:
		#if t - start > 60:
		#	print "over time"
			#test.write(str(f) + '\n')
			#test.write("time break \n")
		#	return sum(times)
				
		#if (len(hit) > 100):
			
		#	print "over length"
		#	test.write(str(f) + '\n')
		#	test.write( "break \n")
		#	return sum(times)

		f
		
	
	#test.close()
	return sum(times)
		
def get_map(prefix):
    return {x[len(prefix):] : y for x, y in globals().items() if x.startswith(prefix)}

def select_best(l_obst):

	global goal, hit_counter, best_score, best_obst
	#times = []
	#global hit_counter
	'''
	count_list = []
	hit_list = []
	tmp = []
	for obs in l_obst[-20:]:

		#times.append(time.clock())
		#r = run_prog_process(obs)
		all_obst = obs
		if (len(obs) < 6):
			all_obst = obs + goal.obstacles
		
		#n_ob = check.get_valid_output(all_obst)
		overlap_c = check.get_overlap_count(all_obst)
		#c  = 50	
		
		r = run_prog_process(all_obst)
		v_x = float(r[2])
		v_z = float(r[3])
		count = int(r[-1])

		f2 = open('./results/rstring.txt', 'r')
		l = f2.readlines()
		re = l[0].strip()
		f2.close()
		
		
		if re[0] == 's':
			#print hit
			if (overlap_c == 0):
				hit_list.append(all_obst)
			
				c = v_x**2 + v_z **2 + count

				count_list.append(c)
	
	if (len(hit_list) > 0):
		print " hit"	
		obst = hit_list[np.argmin(count_list)]

		path = './tests/best_from_select.txt'
		write_setup(obst, path)

	'''

	if (len(best_obst)):
		path = './tests/best_from_select.txt'
		write_setup(best_obst, path)

	return best_obst, time.clock()
		
		

class Optimizer():
	
	def get_layout(self, n_obs):
		
		global goal, hit, hit_cost, get_params, cost_function, best_obst, best_score, start

		# reset
		hit = []
		hit_cost = []
		goal = read_goal_setup()
        	best_score = 100
        	best_obst = []

  		params_map = get_map('get_params_part_')
  		bounds_map = get_map('get_bounds_part_')
  		cost_map = get_map('cost_part_')

		cost_function = cost_map[str(n_obs)]
  		get_params = params_map[str(n_obs)]
  		bounds = bounds_map[str(n_obs)]()

		t = time.clock()
		start = time.time()
		obs = []

		files = glob.glob('./dyn_results/*')
		for f in files:
			os.remove(f)
		total_time = 0.0
		total_time2 = 0.0
		
		# clear up debug file
		if os.path.exists("./tests/break_test.txt"):
			os.remove("./tests/break_test.txt")

		# use your favourite opt algo to run

		total_time = run(method_differential_evolution())
		#total_time = run(method_slsqp_high())
		#total_time = run(method_slsqp_low())
		#total_time = run(method_cg_eps_high())
		#total_time = run(method_cg_eps_low())
		#total_time = run(method_cma())
		
		
		if(len(best_obst) <= 0):
			
			#path = "./tests/best_setup.txt"
			
			#write_setup(best_obst, path)
			total_time = run(method_differential_evolution())
			
		#else :
		#	total_time = run(method_differential_evolution())
		#	total_time2 = run(method_slsqp_low())		
		
		#dir = r'./dyn_results'
		
		'''
		for dirpath, dirnames, files in os.walk(dir):
			while not files:
				total_time = run(method_differential_evolution())
				total_time2 = run(method_slsqp_low())
		'''
		#print(str(len(hit)))
		#print ("total time : ", total_time + total_time2)
		#print ("total time : ", total_time )
		#print ("total time 2 : ", total_time2 )
		#print ("select time : ", time_s )

		#print("run time : ", t)

  		#print("hit count :", len(hit))
  		#print("min cost test len : ", len(hit_cost))
  #write_setup(obs)
		
  #run_prog_process(strres)'''
