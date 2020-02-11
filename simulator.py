#!/usr/bin/env python
# This simulator uses a beta version of CGF which takes into account the weight of the motor's arms. 
# After installing dependencies (matplotlib, numpy), run this program in a linux terminal with "python simulator.py"
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
DISCHARGE_CAPACITY = .8
LOSS = 1.1 # factor multiplied by current draw to approximate loss of efficiency through wire resistance, etc.
NOMINAL_VOLTAGE = 12.*3.7

def rad(th): # convert input (degrees) into radians
	if type(th) is list:
		return np.array(th)*(np.pi/180)
	return th*(np.pi/180)

class Drone:
	# Drone is assumed to be stable and hovering in place
	def __init__(self, cgx_pl, cgy_pl, M_pl=1, init_th_deg=[45,45,45,45], thrust_ac=None):
		#																				"thrust accuracy" depicts how accurately thrust is calculated
		# Given Fields
		self.M_pl = M_pl # payload mass
		self.M_arms = .92*4 # .92kg / full arm assembly
		self.M_tot = DRONE_MASS + self.M_pl # DRONE_MASSkg for 2 batts / no payload 
		self.M_bod = self.M_tot - self.M_arms - self.M_pl
		self.R = .5207+.0635 # .5207m
		self.R_armcg = .403 # .403m
		self.th_deg = init_th_deg
		self.max_motor_thrust = 7*.9*.8 # 7kg for KDE 220kv, 50v, dual-blade
		self.batt_Ah = 22 # only 80% of this is used - to simulate reality
		if not (thrust_ac is None): # if thrust_ac is None, it will do the faster algorithm
			self.thrust_ac = float(thrust_ac)
		else:
			self.thrust_ac = None
		self.quiet = False
		# In application, thrusts will be given and we solve for cg_pl. Here, cg_pl is given and we solve for thrusts.

		# Derived Fields
		self.cgx = None
		self.cgy = None
		self.cgx_pl = cgx_pl
		self.cgy_pl = cgy_pl
		self.calcCGs()
		self.thrusts = None
		self.power = None
		self.power_tot = None
		self.current = None
		self.current_tot = None
		self.hover_time = None
		self.calcThrusts()
		self.stable = None

		self.pltInfo = {}

	def quietMode(self, on=True):
		self.quiet = on

	def setR(self, r):
		self.r = R

	def unmorph(self):
		self.th_deg = [45,45,45,45]
		self.calcCGs()
		self.calcThrustsFast()

	# CheckPL = check if payload position is feasible
	def morph(self, checkPL=False):
		self.th_deg = self.matchCG()
		self.calcCGs()
		if checkPL:
			return self.calcThrusts(justCheck=True)
		else:
			self.calcThrusts()

	def isStable(self):
		thrusts = np.array(self.thrusts)
		positive = (thrusts > .01*self.max_motor_thrust*2).all()
		withinRange = (thrusts < .99*self.max_motor_thrust*2).all()
		return (positive and withinRange)

	# This function is used to ensure correctness and accuracy of the simulation
	def assertPhysics(self):
		r = 4
		# Masses
		M_tot = self.M_pl + self.M_arms + self.M_bod
		M_tot = round(float(str(M_tot)), 5)
		self.M_tot = round(float(str(self.M_tot)), 5)		
		assert (float(str(M_tot)) == float(str(self.M_tot))), "Mass asertion Failed: "+str(M_tot)+" != "+str(self.M_tot)
		# CG
		th = rad( self.th_deg )
		x = np.array( [np.cos(th[0]), -np.cos(th[1]), -np.sin(th[2]), np.sin(th[3])] )
		y = np.array( [np.sin(th[0]), -np.sin(th[1]), np.cos(th[2]), -np.cos(th[3])] )
		cgx = round( (np.sum(x*self.R_armcg*.25*self.M_arms) + self.cgx_pl*self.M_pl)/self.M_tot, r ) 
		cgy = round( (np.sum(y*self.R_armcg*.25*self.M_arms) + self.cgy_pl*self.M_pl)/self.M_tot, r )
		cgx_rec = round(self.cgx,r)
		cgy_rec = round(self.cgy,r)
		assertMsg = "CG Assertion Failed:\nCalculated: "+str(cgx)+", "+str(cgy)+"\nself: "+str(cgx_rec)+", "+str(cgy_rec)
		assert ((cgx==cgx_rec) and (cgy==cgy_rec)), assertMsg
		# Static Equilibreum
		if (self.thrusts is None): return
		a = np.around(self.getCoF(th_deg=self.th_deg, m=self.thrusts), 2)
		b = np.around(np.array([self.cgx, self.cgy]), 2)
		assert np.all(np.equal(a, b)), "Static Equilibreum assertion Failed : "+str(a)+", "+str(b)

	def getCoF(self, th_deg=None, m=None): # returns center of force of drone, whether that be thrusts or gravity
		if (m is None):
			m = self.thrusts
		if (th_deg is None):
			th_deg = self.th_deg
		th = np.array(th_deg)*(np.pi/180)
		x = self.R*np.array( [np.cos(th[0]), -np.cos(th[1]), -np.sin(th[2]), np.sin(th[3])] )
		y = self.R*np.array( [np.sin(th[0]), -np.sin(th[1]), np.cos(th[2]), -np.cos(th[3])] )
		return np.array( [np.dot(m,x)/np.sum(m), np.dot(m,y)/np.sum(m)] ) #x,y

	def setPL(self,x=None,y=None,m=None,calcThrusts=True):
		if x is None:
			x = self.cgx_pl
		if y is None:
			y = self.cgy_pl
		if m is None:
			m = self.M_pl

		self.M_pl = m # payload mass
		self.M_tot = DRONE_MASS + self.M_pl # DRONE_MASSkg for 2 batts / no payload 
		self.M_bod = self.M_tot - self.M_arms - self.M_pl
		
		self.cgx_pl = x
		self.cgy_pl = y
		self.calcCGs()
		if calcThrusts:
			self.calcThrusts()
		#self.assertPhysics()

	# Simply checks if PL is able to be held 
	def checkPL(self,x,y):
		self.cgx_pl = x
		self.cgy_pl = y
		self.calcCGs()
		return self.calcThrusts(justCheck=True)

	def calcCGs(self): # calculate total and payload cg
		self.cgx, self.cgy = self.predictCG(new_th_deg=self.th_deg) # get CG based on known payload - not motor speeds

	def predictCG(self, new_th_deg):  # using payload cg and weighted arms, predict cg of new th
		th = rad(new_th_deg)
		arms_sigma_x = self.R_armcg*( np.cos(th[0]) - np.cos(th[1]) - np.sin(th[2]) + np.sin(th[3]) )
		arms_sigma_y = self.R_armcg*( np.sin(th[0]) - np.sin(th[1]) + np.cos(th[2]) - np.cos(th[3]) )
		cgx_pred = (1./self.M_tot)*( .25*self.M_arms*arms_sigma_x + self.M_pl*self.cgx_pl )
		cgy_pred = (1./self.M_tot)*( .25*self.M_arms*arms_sigma_y + self.M_pl*self.cgy_pl )
		return cgx_pred, cgy_pred

	def matchCG(self, cgx=None, cgy=None, nearest=False):  # find the th required to make motor speeds as close to equal as possible
		start = time.time()
		th = [rad(45)]*4
		if nearest:
			th = rad(self.th_deg)
		min_dist = .001
		converged = False
		i = 0
		last_dist = 0
		self.calcCGs()
		while( not converged ):
			i = i+1
			# Calculate gradient
			s = np.sin(th)
			c = np.cos(th)
			a_ = ((self.M_pl*self.cgx_pl + (self.M_arms*self.R_armcg*(c[0] - c[1] - s[2] + s[3]))/4)/self.M_tot - (self.R*(c[0] - c[1] - s[2] + s[3]))/4)
			b_ = (self.M_arms*self.R_armcg*(c[2] - c[3] + s[0] - s[1]))
			c_ = ((self.M_pl*self.cgy_pl + b_/4)/self.M_tot - (self.R*(c[2] - c[3] + s[0] - s[1]))/4)
			d_ = self.M_arms*self.R_armcg/(4*self.M_tot)
			grad = np.array( [ 2*((self.R*s[0])/4 - (d_*s[0]))*a_ - 2*((self.R*c[0])/4 - (d_*c[0]))*c_,
				2*((self.R*c[1])/4 - (d_*c[1]))*c_ - 2*((self.R*s[1])/4 - (d_*s[1]))*a_,
				2*((self.R*c[2])/4 - (d_*c[2]))*a_ + 2*((self.R*s[2])/4 - (d_*s[2]))*c_,
				- 2*((self.R*c[3])/4 - (d_*c[3]))*a_ - 2*((self.R*s[3])/4 - (d_*s[3]))*c_] )
			grad[np.abs(grad) < 1e-5] = 0
			# Move against direction of gradient
			th = np.add(th, -.01*np.sign(grad))
			th = np.clip(th, rad(MIN_ANGLE), rad(MAX_ANGLE))
			ct = self.getCoF(th*(180/np.pi), m=[1,1,1,1]) # center of thrust
			cgx,cgy = self.predictCG(th*(180/np.pi))
			# Check if converged
			dist = np.linalg.norm(np.subtract(ct, np.array(cgx,cgy)))
			converged = (dist < min_dist) or (last_dist == dist) or (i >= 100) # it reaches point, or it reached angle limits, or cg is out of solution set (50 was the max seen 1/7/19)
			last_dist = dist # if dist wasn't changed, we can't do any better
		if not self.quiet:
			if dist <= min_dist:
				print(">Morphing successfully converged")
			else:
				print(">Morphing could not succesfully converge")
			print(">Accuracy: "+str(round(1000.*dist,1))+" mm")
			print( ">Time used: "+str(int(round(1000*(time.time()-start))))+" ms")
			print(">Iterations: "+str(i))
		#print("MatchCG reached distance "+str(dist)+"m in "+str(i)+" iterations.")
		return th*(180./np.pi)

	# Less efficient version of calcThrustsFast (SOON TO BE DEPRECATED)
	def calcThrusts(self, justCheck=False):
		# input: motor position, vehicle CG --- output: thrust of each motor pair
		# For a coaxial quad, this problem is underconstrained (3 contrainsts, 4 unknowns).
		# Therefore, we use the last thrust as a variable and toggle it to find the argmin of the maximum of the 4 thrusts (2 motors per thrust)
		if (justCheck is False) and (self.thrust_ac is None):
			return self.calcThrustsFast()
		if not self.quiet:
			print("Using slow calcthrusts")
		th = rad(self.th_deg)
		x = self.R*np.array( [np.cos(th[0]), -np.cos(th[1]), -np.sin(th[2]), np.sin(th[3])] ) # x-coordinates of motors
		y = self.R*np.array( [np.sin(th[0]), -np.sin(th[1]), np.cos(th[2]), -np.cos(th[3])] ) # y-""    ""
		a_ = (x[0]*y[1] - x[1]*y[0] - x[0]*y[2] + x[2]*y[0] + x[1]*y[2] - x[2]*y[1])
		M_tot = self.M_tot
		cgx = self.cgx
		cgy = self.cgy
		smallestMaxT = self.max_motor_thrust*2 + 1
		T = np.array([None, None, None, None])
		T_best = None
		for i in np.arange(0,self.M_tot,self.M_tot/self.thrust_ac):
			T[3] = i
			T[0] = -(M_tot*cgy*x[1] - M_tot*cgy*x[2] - M_tot*cgx*y[1] + M_tot*cgx*y[2] - M_tot*x[1]*y[2] + M_tot*x[2]*y[1] + T[3]*x[1]*y[2] - T[3]*x[2]*y[1] - T[3]*x[1]*y[3] + T[3]*x[3]*y[1] + T[3]*x[2]*y[3] - T[3]*x[3]*y[2])/a_ 
			T[1] = (M_tot*cgy*x[0] - M_tot*cgy*x[2] - M_tot*cgx*y[0] + M_tot*cgx*y[2] - M_tot*x[0]*y[2] + M_tot*x[2]*y[0] + T[3]*x[0]*y[2] - T[3]*x[2]*y[0] - T[3]*x[0]*y[3] + T[3]*x[3]*y[0] + T[3]*x[2]*y[3] - T[3]*x[3]*y[2])/a_ 
			T[2] = -(M_tot*cgy*x[0] - M_tot*cgy*x[1] - M_tot*cgx*y[0] + M_tot*cgx*y[1] - M_tot*x[0]*y[1] + M_tot*x[1]*y[0] + T[3]*x[0]*y[1] - T[3]*x[1]*y[0] - T[3]*x[0]*y[3] + T[3]*x[3]*y[0] + T[3]*x[1]*y[3] - T[3]*x[3]*y[1])/a_
			maxT = np.max(T)
			if (T>=0).all() and (T<self.max_motor_thrust*2).all() and (maxT < smallestMaxT):
				if justCheck:
					return True # signifying that flight is possible
				smallestMaxT = maxT
				T_best = copy.deepcopy(T)
		self.thrusts = T_best
		self.assertPhysics()
		if T_best is None:
			return False
		self.calcEnergyStats()

		# Returns 4 thrusts; 1 for each pair of motors. The resulting thrusts are as close together as possible
	def calcThrustsFast(self, justCheck=False):
		# input: motor position, vehicle CG --- output: thrust of each motor pair
		# For a coaxial quad, this problem is underconstrained (3 contrainsts, 4 unknowns).
		# Therefore, we use the last thrust as a variable and toggle it to find the argmin of the maximum of the 4 thrusts (2 motors per thrust)
		th = rad(self.th_deg)
		x = self.R*np.array( [np.cos(th[0]), -np.cos(th[1]), -np.sin(th[2]), np.sin(th[3])] ) # x-coordinates of motors
		y = self.R*np.array( [np.sin(th[0]), -np.sin(th[1]), np.cos(th[2]), -np.cos(th[3])] ) # y-""    ""
		a_ = (x[0]*y[1] - x[1]*y[0] - x[0]*y[2] + x[2]*y[0] + x[1]*y[2] - x[2]*y[1])
		M_tot = self.M_tot
		cgx = self.cgx
		cgy = self.cgy
		smallestMaxT = self.max_motor_thrust*2 + 1
		def func(t): # 0 < x < M_tot
			T = np.array([None, None, None, None])
			T[3] = t
			T[0] = -(M_tot*cgy*x[1] - M_tot*cgy*x[2] - M_tot*cgx*y[1] + M_tot*cgx*y[2] - M_tot*x[1]*y[2] + M_tot*x[2]*y[1] + T[3]*x[1]*y[2] - T[3]*x[2]*y[1] - T[3]*x[1]*y[3] + T[3]*x[3]*y[1] + T[3]*x[2]*y[3] - T[3]*x[3]*y[2])/a_ 
			T[1] = (M_tot*cgy*x[0] - M_tot*cgy*x[2] - M_tot*cgx*y[0] + M_tot*cgx*y[2] - M_tot*x[0]*y[2] + M_tot*x[2]*y[0] + T[3]*x[0]*y[2] - T[3]*x[2]*y[0] - T[3]*x[0]*y[3] + T[3]*x[3]*y[0] + T[3]*x[2]*y[3] - T[3]*x[3]*y[2])/a_ 
			T[2] = -(M_tot*cgy*x[0] - M_tot*cgy*x[1] - M_tot*cgx*y[0] + M_tot*cgx*y[1] - M_tot*x[0]*y[1] + M_tot*x[1]*y[0] + T[3]*x[0]*y[1] - T[3]*x[1]*y[0] - T[3]*x[0]*y[3] + T[3]*x[3]*y[0] + T[3]*x[1]*y[3] - T[3]*x[3]*y[1])/a_
			return T
		def dfunc(t, d=.001):
			y2 = np.max( func(t+d) )
			y1 = np.max( func(t) )
			return np.sign( y2 - y1 )

		st = time.time()
		# O(logn) algorithm for reaching inflection point of 
		iterations = 0
		thrust_accuracy = .01
		T3 = .9*M_tot
		last_move = T3
		while last_move > thrust_accuracy:
			last_move = last_move/2
			positive = dfunc(T3) > 0 # works
			if positive:
				T3 = T3 - last_move
			else:
				T3 = T3 + last_move
			iterations = iterations + 1

		T = func(T3)

		if ( (T>=0).all() and (T<self.max_motor_thrust*2).all() ):
			self.stable = True
		else:
			self.stable = False
			print("No viable thrust found")
		self.thrusts = copy.deepcopy(T)

		self.assertPhysics()
		self.calcEnergyStats()

	# KDE-CF185-DP
	# input: thrusts of motor pairs (kg) --- output: power of motor pairs (W)
	def powerModel(self, kg):
		return -1.6073*np.power(kg,3) + 29.092*np.power(kg,2) + 42.516*(kg) + 15.204
	
	# input: thrusts of motor pairs (kg) --- output: power of motor pairs (W)
	def currentModel(self, kg):
		return -.0347*np.power(kg,3) + .6296*np.power(kg,2) + .9184*(kg) + .3413

	# input: thrusts of motor pair (kg) --- output: average rpm of motor pair (W)
	#def rpmModel(self, kg):
	#	return -1.4194*np.power(kg,4) + 31.517*np.power(kg,3) - 290.84*np.power(kg,2) + 1901.3*kg + 1401.8

	def calcEnergyStats(self, thrust = None):
		if thrust is None:
			thrust = self.thrusts
		#self.power = np.multiply( self.powerModel(thrust/2.) ,2)
		#self.power_tot = int(round(sum(self.power)))#*LOSS
		self.current = np.multiply( self.currentModel(thrust/2.) ,2) * LOSS
		self.current_tot = round(sum(self.current), 3)
		self.power = self.current*NOMINAL_VOLTAGE
		self.power_tot = self.current_tot*NOMINAL_VOLTAGE
		#hover_min = (DISCHARGE_CAPACITY*self.batt_Ah/(self.power_tot/NOMINAL_VOLTAGE))*60. # (Ah/(Watts/Volts))*(min/hour)
		hover_min = 60.*(DISCHARGE_CAPACITY*self.batt_Ah)/self.current_tot
		hover_sec = (hover_min%1)*60.
		if int(round(hover_sec)) >= 60:
			hover_sec = 0
			hover_min = hover_min + 1
		self.hover_time = str(int(hover_min))+"min "+str(int(round(hover_sec)))+"sec"
		return self.power

	def printStuff(self):
		print("------------------")
		print("th_deg",self.th_deg)
		print("cgx_pl, cgy_pl",self.cgx_pl,self.cgy_pl)
		print("cgx, cgy",self.cgx, self.cgy)
		print("thrusts",self.thrusts)
		print("sum(thrusts)",np.sum(self.thrusts))
		print("power",self.power)
		print("sum(power)",self.power_tot)
		print("M_tot", self.M_tot)
		print("M_pl", self.M_pl)
		print("M_arms",self.M_arms)
		print("Hover time:"+self.hover_time)
		print("------------------")

	def annotateDrone(self):
		th = rad(np.array(self.th_deg)+10)
		x = self.R*np.array( [np.cos(th[0]), -np.cos(th[1]), -np.sin(th[2]), np.sin(th[3])] )
		y = self.R*np.array( [np.sin(th[0]), -np.sin(th[1]), np.cos(th[2]), -np.cos(th[3])] )
		scale = .8
		for i in range(4):
			plt.text(scale*x[i]*UNIT_MULT, scale*y[i]*UNIT_MULT, str(round(self.thrusts[i],2))+"kg")
		#plt.text(-self.R/5,-self.R/2,str(self.M_pl)+"kg payload at ("+str(self.cgx_pl)+", "+str(self.cgy_pl)+")")
		plt.text(.6*self.cgx_pl*UNIT_MULT, .9*self.cgy_pl*UNIT_MULT, str(self.M_pl)+"kg payload at ("+str(self.cgx_pl)+", "+str(self.cgy_pl)+")")
		plt.plot(self.cgx_pl*UNIT_MULT, self.cgy_pl*UNIT_MULT, 'ro')
		plt.plot(self.cgx*UNIT_MULT, self.cgy*UNIT_MULT, 'bo')
		plt.text(self.cgx*UNIT_MULT, self.cgy*UNIT_MULT, "cg")

	def annotateMotors(self):
		th = rad(np.array(self.th_deg)+10)
		x = self.R*np.array( [np.cos(th[0]), -np.cos(th[1]), -np.sin(th[2]), np.sin(th[3])] )
		y = self.R*np.array( [np.sin(th[0]), -np.sin(th[1]), np.cos(th[2]), -np.cos(th[3])] )
		scale = .8
		objs = []
		for i in range(4):
			#plt.text(scale*x[i], scale*y[i], str(round(self.thrusts[i],2))+"kg")
			objs.append( plt.text(scale*x[i]*UNIT_MULT, scale*y[i]*UNIT_MULT, str(i+1)) )
		return objs


	def annotateAngles(self):
		th = rad(np.array(self.th_deg))
		x = self.R*np.array( [np.cos(th[0]), -np.cos(th[1]), -np.sin(th[2]), np.sin(th[3])] )
		y = self.R*np.array( [np.sin(th[0]), -np.sin(th[1]), np.cos(th[2]), -np.cos(th[3])] )
		scale = .5
		for i in range(4):
			plt.text(scale*x[i]*UNIT_MULT, scale*y[i]*UNIT_MULT, str(round(self.th_deg[i],1))+" Deg")

	def morphMessage(self):
		morphingMessage = "Has not reached morphing limit"
		if np.any(self.th_deg+.1 > MAX_ANGLE) or np.any(self.th_deg-.1 < MIN_ANGLE):
			morphingMessage = "Reached morphing limit"
		plt.suptitle(morphingMessage, ha='center', va='top')

	def plotDrone(self, th_deg=None, col='g', pt='o'):
		if th_deg is None:
			th_deg = self.th_deg
		plt.gca().set_aspect('equal', adjustable='box')
		th_deg = np.array(th_deg) # theta
		th = th_deg*(np.pi/180)
		x = self.R*np.array( [np.cos(th[0]), -np.cos(th[1]), -np.sin(th[2]), np.sin(th[3])] )
		y = self.R*np.array( [np.sin(th[0]), -np.sin(th[1]), np.cos(th[2]), -np.cos(th[3])] )
		objs = []
		for pair in zip(x,y): # arms
			lineObj, = plt.plot([0, pair[0]*UNIT_MULT], [0, pair[1]*UNIT_MULT], col)
			objs.append(lineObj)
		objs.append( plt.plot(x*UNIT_MULT,y*UNIT_MULT, col+pt)[0] ) # motors
		objs.append( plt.plot(0,0, col+pt)[0] ) # center
		plt.xlabel(UNITS)
		plt.ylabel(UNITS)
		return objs
