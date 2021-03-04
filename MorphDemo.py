#!/usr/bin/env python

# MorphDemo
# Jordan Coult

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
import matplotlib.patches as patches
import numpy as np
import sys
import copy
import time
from simulator import Drone
MAX_ANGLE_CHANGE = 30 # max mechanical angle deviation frome normal quadcopter arm position (degrees)
MIN_ANGLE = 45 - MAX_ANGLE_CHANGE
MAX_ANGLE = 45 + MAX_ANGLE_CHANGE
DRONE_MASS = 15.26
UNIT_MULT = 100.
UNITS = 'cm'

init_pl_mass = 4.

# maybe later: use scalarmappable images
plt.clf() # clear current figure

plt.subplot(1,2,2)
plt.gca().set_aspect('equal', adjustable='box')
plt.gca().grid(False)
plt.tick_params(axis='both', bottom=False, left=False, labelleft=False, labelbottom=False)
plt.gca().axis('off')

plt.subplot(1,2,1)
plt.subplots_adjust(bottom=0.25)
d_background = Drone(M_pl=1, cgx_pl=0,cgy_pl=0)
d = Drone(M_pl=init_pl_mass, cgx_pl=.13, cgy_pl=0)
main_ax = plt.gca()
d.pltInfo['first_i'] = True # first drawing of plot info

# REMOVEABLE GRAPHICS
obj_groups = ['pl_objs', 'cg_objs'] # payload, center of gravity
for group in obj_groups:
	d.pltInfo[group] = []
# reference drone
d.pltInfo['ref_objs'] = d_background.plotDrone(th_deg=[45,45,45,45], col='c', pt=',')
# actual drone
d.pltInfo['drone_objs'] = d.plotDrone(col='b')
# pl x
d.pltInfo['pl_objs'].append( plt.plot(d.cgx_pl*UNIT_MULT, d.cgy_pl*UNIT_MULT, 'rx')[0] )
# pl square
rect = patches.Rectangle(((d.cgx_pl-.05)*UNIT_MULT,(d.cgy_pl-.05)*UNIT_MULT), .1*UNIT_MULT, .1*UNIT_MULT, linewidth=1, edgecolor='r',facecolor='none') 
d.pltInfo['pl_objs'].append( rect )
main_ax.add_patch(rect)
# pl text
#d.pltInfo['pl_objs'].append( plt.text(d.cgx_pl-.15, d.cgy_pl+.05, "payload") )
# cg pt
d.pltInfo['cg_objs'].append( plt.plot(d.cgx*UNIT_MULT, d.cgy*UNIT_MULT, 'ko')[0] )
# cg text

d.pltInfo['morph_mode'] = False

plt.xlim([-1*UNIT_MULT,1*UNIT_MULT])
plt.ylim([-1*UNIT_MULT,1*UNIT_MULT])

def percentThrust(kg):
	percent = int(round(kg/(d.max_motor_thrust*2)*100))
	if (percent < 0) or (percent > 100):
		return "--%"
	else:
		return str(percent)+"%"

def prompt(s):
	plt.sca(main_ax)
	print(s)
	plt.gcf().suptitle(s, ha='center', va='top')
	#plt.title(s)

# Every removeable group must be an array 
def removeObjects(group):
	full_group_key = group+"_objs"
	for obj in d.pltInfo[ full_group_key ]:
		obj.remove()
	d.pltInfo[ full_group_key ] = []

def updateTextBox():
	plt.subplot(1,2,2)
	if not d.pltInfo['first_i']:
		removeObjects('info')
	thrust = np.round_(np.array(d.thrusts).astype(np.double), 1)
	th = np.round_(np.array(d.th_deg)-45, 1)
	cax = plt.gca()
	maneuverability = int(round((1-np.max(thrust)/(d.max_motor_thrust*2))*100))
	if not d.isStable():
		maneuverability = 0
	d.pltInfo['info_objs'] = [cax.text(.05,.95, 
			'                               '+str(maneuverability)+'%\n'
			'                                      '+str(int(round(d.power_tot)))+'W\n'+
			'                      '+d.hover_time+'\n\n'+
			'\n'+
			'                         '+percentThrust(thrust[0])+' / '+str(th[0])+'deg\n'+
			'                         '+percentThrust(thrust[2])+' / '+str(th[2])+'deg\n'+
			'                         '+percentThrust(thrust[1])+' / '+str(th[1])+'deg\n'+
			'                         '+percentThrust(thrust[3])+' / '+str(th[3])+'deg\n\n'+
			'                         '+str(round(DRONE_MASS,1))+'kg\n'+
			'                12S2P, '+str(d.batt_Ah)+'Ah\n'+
			'                       '+str(int(round(d.R*UNIT_MULT)))+UNITS+'\n'
			'                                  '+str(MAX_ANGLE_CHANGE)+'deg',
			ha='left', va='top', color='grey'),
		cax.text(.05,.95,'Maneuverability:\n'+
			'Power Consumption: \n'+
			'Hover time: \n\n'+
			'Motor Thrusts / Morph Angles\n'+
			'Top right:   \n'+
			'Top left:    \n'+
			'Bottom left:  \n'+
			'Bottom right: \n\n'+
			'Vehicle Mass: \n'+
			'Battery: \n'+
			'Arm length: \n'
			'Morph angle limit: ',
			fontweight='semibold', color='grey', va='top', ha='left')
		]
	plt.subplot(1,2,1)
	d.pltInfo['first_i'] = False
	plt.draw()

def reportStability():
	if d.isStable():
		prompt("Drone is in stable hover")
	else:
		prompt("Drone is unstable. Morph or change payload")

def morph(unmorph=False):
	if unmorph:
		d.unmorph()
	else:
		d.morph()
	plt.sca(main_ax)
	removeObjects('drone')
	removeObjects('cg')
	# Redraw
	d.pltInfo['cg_objs'].append( main_ax.plot(d.cgx*UNIT_MULT, d.cgy*UNIT_MULT, 'ko')[0] )
	d.pltInfo['drone_objs'] = d.plotDrone(col = 'b')
	# Update
	updateTextBox()
	reportStability()
	plt.draw()

# CALLBACKS 

def pl_pos_update(event):
	prompt('Select desired payload position')
	pt = np.asarray(plt.ginput(timeout=-1))[0] # = coordinates of user click
	plt.sca(main_ax)
	# Set vals
	d.setPL(x=pt[0]/UNIT_MULT, y=pt[1]/UNIT_MULT)
	if d.pltInfo['morph_mode']:
		morph()
	# Remove
	removeObjects('cg')
	removeObjects('pl')
	# Redraw
	d.pltInfo['pl_objs'].append( main_ax.plot(pt[0], pt[1], 'rx')[0] )
	rect = patches.Rectangle((pt[0]-.05*UNIT_MULT,pt[1]-.05*UNIT_MULT), .1*UNIT_MULT, .1*UNIT_MULT, linewidth=1, edgecolor='r',facecolor='none')
	d.pltInfo['pl_objs'].append( rect )
	main_ax.add_patch(rect)
	d.pltInfo['cg_objs'].append( main_ax.plot(d.cgx*UNIT_MULT, d.cgy*UNIT_MULT, 'ko')[0] )
	reportStability()
	updateTextBox()
	plt.draw()

def pl_mass_update(val):
	val = round(val,1)
	# Remove
	removeObjects('cg')
	# Set vals
	d.setPL(m=val)
	if d.pltInfo['morph_mode']:
		morph()
	# Redraw
	d.pltInfo['cg_objs'].append( main_ax.plot(d.cgx*UNIT_MULT, d.cgy*UNIT_MULT, 'ko')[0] )
	updateTextBox()
	reportStability()
	plt.draw()

morph_label = "Morphing Drone"
def morph_radio_callback(label):
	if label == morph_label:
		morph()
		d.pltInfo['morph_mode'] = True
	else:
		morph(unmorph=True)
		d.pltInfo['morph_mode'] = False


ax_plPos = plt.axes([1-.06-.25,.01,.25,.05]) # left, bottum, width, height
b_plPos = Button(ax_plPos, "Set Position")
s = b_plPos.on_clicked(pl_pos_update)

ax_plM = plt.axes([1-.06-.25, .08, .25, .05])
sl = Slider(ax_plM, 'Payload Settings', 0, 20, valinit=init_pl_mass, valstep=.1, valfmt='%1.1fkg')
sl.label.set_ha('left') 
sl.label.set_y(1.7) # explain on stackexchange question
sl.on_changed(pl_mass_update)

ax_morph = plt.axes([.01,.01,.15,.15])#plt.axes([.89,.01,.1,.05])
ax_morph.axis('off')
radio_morph = RadioButtons(ax_morph, (morph_label, 'Non-morphing Drone'), active=1)
radio_morph.on_clicked(morph_radio_callback)

updateTextBox()
prompt("Change payload mass & position, analyze flight stats, or morph")

# LEGEND 

main_ax.legend( ( d.pltInfo['cg_objs'][0], d.pltInfo['drone_objs'][0], d.pltInfo['pl_objs'][1] ), 
	( 'CG', 'Drone', 'Payload' ))

plt.show()

# add a remaining maneuverability % in info box
# make custom text so payload mass value is left of slider and slider is .01 from right
# show payload distance limit for nonmorphed drone. If someone presses outside it, it says "drone unstable - press morph"
# show drone info, like thrusts, in visual representation such as bar graph with spread shown 

# (1) user defines payload, program finds motorspeeds and cg
# set position and weight of payload
# have a morph and revert button
# have box showing general info of drone (weight, lengths, cg, angles)
# have another box showing performance (thrusts, flight time, instantaneous power, )

# (2) user defines motor speeds, program detects cg and can morph to it

