#!/usr/bin/env python
# Jordan Coult
# A collection of various plots, some outdated, which build upon the Drone class to demonstrate performance characteristics

from simulator import Drone
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
import matplotlib.patches as patches
import numpy as np
import sys
import copy
import time
MAX_ANGLE_CHANGE = 30 # max mechanical angle deviation frome normal quadcopter arm position (degrees)
MIN_ANGLE = 45 - MAX_ANGLE_CHANGE
MAX_ANGLE = 45 + MAX_ANGLE_CHANGE
DRONE_MASS = 14
UNIT_MULT = 100.
UNITS = 'cm'

def rad(th): # convert input (degrees) into radians
	if type(th) is list:
		return np.array(th)*(np.pi/180)
	return th*(np.pi/180)

# Simple example of the drone's performance before and after morphing
def example_sim():
	d = Drone(M_pl=2, cgx_pl=1, cgy_pl=1)
	d.printStuff()

	plt.subplot(1,2,1)
	d.plotDrone(th_deg = d.th_deg)
	d.annotateDrone()
	plt.plot(d.cgx,d.cgy,'go')
	plt.title("Before Morphing (green)")
	plt.figtext(0.01,0.02, "Instantaneous Power: "+str(d.power_tot)+"W, Max Hover Time: "+d.hover_time)
	plt.subplot(1,2,2)
	d.plotDrone(th_deg = d.th_deg)

	d.morph()
	d.printStuff()

	d.plotDrone(th_deg = d.th_deg, col='r')
	d.annotateDrone()
	x,y = d.getCoF(m=[1,1,1,1])
	plt.plot(x,y,'ro')
	plt.title("After Morphing (red)")
	plt.figtext(0.55,0.02, "Instantaneous Power: "+str(d.power_tot)+"W, Max Hover Time: "+d.hover_time)
	plt.show()

def efficiency1():
	d = Drone(M_pl=5, cgx_pl=0, cgy_pl=0, thrust_ac=1000)
	d_morph = Drone(init_th_deg=[75,15,15,75], M_pl=5, cgx_pl=0, cgy_pl=0, thrust_ac=1000)
	assert(d.M_pl == d_morph.M_pl)
	print("Starting")
	amtData = 17
	maxD = 2.5
	data_nonmorph = np.empty([amtData,2])
	data_morph = np.empty([amtData,2])
	i=0
	for D in np.arange(0,maxD, maxD/amtData):
		print(i)
		print('a')
		d.setPL(0,D)
		if d.thrusts is None:
			data_nonmorph[i,:] = [d.cgy_pl, None]
		else:
			data_nonmorph[i,:] = [d.cgy_pl, d.power_tot]
		print('b')
		d_morph.setPL(0,D)
		d_morph.morph()
		if d_morph.thrusts is None:
			data_morph[i,:] = [d_morph.cgy_pl, None]
		else:
			data_morph[i,:] = [d_morph.cgy_pl, d_morph.power_tot]
		i = i+1
	# Plot Points
	plt.plot(data_nonmorph[:,0], data_nonmorph[:,1], 'bo',
		data_morph[:,0], data_morph[:,1], 'go')
	plt.legend(('Without Morphing', 'With Morphing'), loc='lower right')
	# Annotate
	for i in range(int(amtData/7),amtData):		
		plt.text(data_nonmorph[i,0], data_nonmorph[i,1]-20, str(data_nonmorph[i,1]))
		plt.text(data_morph[i,0], data_morph[i,1]-20, str(data_morph[i,1]))
	# Label
	plt.xlabel("Distance from Drone's Center to Payload (meters)")
	plt.ylabel("Total Power Consumption of Motors (Watts)")
	# Plot Lines
	plt.plot(data_nonmorph[:,0], data_nonmorph[:,1], 'b-',
		data_morph[:,0], data_morph[:,1], 'g-')
	plt.title("Total Power Consumption with Lopsided "+str(d.M_pl)+"kg Payload")
	# Difference Bars
	'''
	y_diff = (data_nonmorph[:,1] - data_morph[:,1])/2
	for i in range(int(amtData/7), amtData):
		if data_morph[i,0] is not None:
			plt.errorbar(data_morph[i,0],
				data_morph[i,1] + y_diff[i],
				yerr=y_diff[i], xerr=0,
				color='k', capsize=10)
	# Difference Annotations
	for i in range(int(amtData/7), amtData):
		plt.text(data_morph[i,0]+.05, data_morph[i,1]+y_diff[i]-3, str(y_diff[i]*2))
	#plt.suptitle("Lopsided Payload Power Consumption")
	'''
	plt.show()

def rpm1():
	d = Drone(init_th_deg=[45,45,45,45], cgx_pl=0, cgy_pl=0, M_pl=1, thrust_ac=1000)
	d_morph = Drone(init_th_deg=[75,15,15,75], cgx_pl=0, cgy_pl=0, M_pl=1, thrust_ac=1000)
	print("Starting")
	amtData = 22
	maxD = 10.
	data_nonmorph = np.empty([amtData,4])
	data_morph = np.empty([amtData,4])
	data_D = np.empty([amtData,1])
	i=0
	for D in np.arange(0,maxD, maxD/amtData):
		print(i)
		print('a')
		d.setPL(0,D)
		data_D[i] = D
		if d.thrusts is None:
			data_nonmorph[i,:] = [None]*4
		else:
			data_nonmorph[i,:] = d.getRPM()
		print('b')
		d_morph.setPL(0,D)
		d_morph.morph()
		if d_morph.thrusts is None:
			data_morph[i,:] = [None]*4
		else:
			data_morph[i,:] = d_morph.getRPM()
		i = i+1
	# Plot Points
	plt.plot(data_D, (data_nonmorph[:,0]+data_nonmorph[:,2])/2, 'b--',
		data_D, (data_nonmorph[:,1]+data_nonmorph[:,3])/2, 'b:',
		data_D, (data_morph[:,0]+data_morph[:,2])/2, 'g--',
		data_D, (data_morph[:,1]+data_morph[:,3])/2, 'g:')
	plt.legend(('No Morphing, Front Motors',
		'No Morphing, Back Motors', 
		'Morphing, Front Motors',
		'Morphing, Back Motors'), loc='lower left')
	# Label
	plt.xlabel("Distance from Drone's Center to Payload (m)")
	plt.ylabel("Average Motor Speed (RPM)")
	plt.title("Motor speeds with Lopsided 1kg Payload")
	plt.show()

# TODO: improve, fix?
def maneuverability1():
	d = Drone(init_th_deg=[45,45,45,45], cgx_pl=0, cgy_pl=0,
		M_pl=1, thrust_ac=100)
	d_morph = Drone(init_th_deg=[75,15,15,75], cgx_pl=0, 
		cgy_pl=0, M_pl=1, thrust_ac=100)
	print("Starting")
	amtData = 22
	maxD = 10.
	data_nonmorph = np.empty([amtData,1])
	data_morph = np.empty([amtData,1])
	data_D = np.empty([amtData,1])
	i=0
	for D in np.arange(0,maxD, maxD/amtData):
		print(i)
		print('a')
		d.setPL(0,D)
		data_D[i] = D
		if d.thrusts is None:
			data_nonmorph[i] = [None]
		else:
			data_nonmorph[i] = round(np.min( 1. - d.thrusts*(1./(7.66*2)) ), 2)
		print('b')
		d_morph.setPL(0,D)
		d_morph.morph()
		if d_morph.thrusts is None:
			data_morph[i,:] = [None]
		else:
			data_morph[i,:] = round(np.min( 1. - d_morph.thrusts*(1./(7.66*2)) ), 2)
		i = i+1
	# Plot Points
	plt.plot(data_D, data_nonmorph, 'bo',
		data_D, data_morph, 'go')
	plt.legend(('Without Morphing',
		'With Morphing'), loc='lower left')
	# Label
	plt.xlabel("Distance from Drone's Center to Payload (m)")
	plt.ylabel("Remaining Thrust of Most Strained Motor (%)")
	plt.title("Remaining Maneuverability with Lopsided 1kg Payload")
	plt.show()

def plotEnvelope():
	d_morph = Drone(cgx_pl=0, cgy_pl=0, M_pl=10,thrust_ac=100)
	d_static = Drone(cgx_pl=0, cgy_pl=0, M_pl=10, thrust_ac=100)
	max_angle = 95.#24. #95.
	angle_numP = max_angle/5#/2#/5 amount of angles of the CG with which to find the max distance
	start_dist = 2.
	dist_accuracy = .003
	max_cg_morph = np.empty([int(angle_numP), 2])
	max_cg_static = np.empty([int(angle_numP), 2])
	for i_angle, angle in enumerate(np.arange(0, max_angle, max_angle/angle_numP)): # angle of cg
		#angle = angle + 38 # debug
		print(str(angle)+"deg")
		# Set the CG to be at the correct angle
		rad_x = np.sin(rad(angle))
		rad_y = np.cos(rad(angle))
		# static distance
		dist = start_dist
		last_move = start_dist
		# Find furthest reachable (works)
		while last_move > dist_accuracy:
			last_move = last_move/2
			canReach = d_static.checkPL(dist*rad_x, dist*rad_y) # works
			if canReach:
				# Then the cg is within reach
				max_cg_static[i_angle,:] = d_static.cgx_pl, d_static.cgy_pl
				dist = dist + last_move
			else:
				dist = dist - last_move
		# morph distance
		dist = start_dist
		last_move = start_dist
		while last_move > dist_accuracy:
			last_move = last_move/2
			d_morph.setPL(dist*rad_x, dist*rad_y, calcThrusts=False)
			canReach = d_morph.morph(checkPL=True)
			if canReach:
				# Then the cg is within reach
				max_cg_morph[i_angle,:] = d_morph.cgx_pl, d_morph.cgy_pl
				dist = dist + last_move
			else:
				dist = dist - last_move

	#Plot
	d_static.plotDrone(col='k')
	lines = plt.plot(max_cg_static[:,0], max_cg_static[:,1], '-b',
		-max_cg_static[:,0], max_cg_static[:,1], '-b',
		max_cg_static[:,0], -max_cg_static[:,1], '-b',
		-max_cg_static[:,0], -max_cg_static[:,1], '-b',
		max_cg_morph[:,0], max_cg_morph[:,1], '-g',
		-max_cg_morph[:,0], max_cg_morph[:,1], '-g',
		max_cg_morph[:,0], -max_cg_morph[:,1], '-g',
		-max_cg_morph[:,0], -max_cg_morph[:,1], '-g')
	plt.legend((lines[0], lines[len(lines)-1]), ('Without Morphing','With Morphing'), loc='lower right')
	plt.title("Furthest CG That Allows Flight (16kg Drone, "+str(d_static.M_pl)+"kg lopsided payload)")
	plt.show()

def test():
	d = Drone(cgx_pl=.49, cgy_pl=.38, M_pl=10,thrust_ac=10000) #.89,.92
	#d_static = Drone(cgx_pl=0, cgy_pl=0, M_pl=10, thrust_ac=10)
	#d = Drone(cgx_pl = 1, cgy_pl = 1, M_pl = 5)
	d.assertPhysics()
	
	plt.subplot(1,2,1)
	plt.title('Before Morphing')
	d.plotDrone()
	d.annotateDrone()	
	print('---Before morphing')
	d.printStuff()

	d.morph()

	plt.subplot(1,2,2)
	plt.title('After Morphing')
	d.plotDrone()
	d.annotateDrone()
	d.annotateAngles()
	print('\n---After morphing')
	d.printStuff()

	d.morphMessage()
	plt.show()


if __name__=="__main__":
	#efficiency1()
	#example_sim()
	#rpm1()
	#maneuverability1()
	#plotEnvelope() #debug: distance log-search algorithm works -- call to checkPL works
	test()
