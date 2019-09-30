#!/usr/bin/env python3
# ====================================================================
# Feedback controler for a Differential drive ground robot 
# ====================================================================
# based on the notes from the lecture "Ground Robot Models" from Erdal Kayacan
# int the course "Control of Mobile Robots- Autumn 2019" 
#
# author: Carlos Hansen Mendoza
# date: september-2019

import sys
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

"""Calculate the next Rho, Alpha and Beta using the desired position and the current robot position"""
def calculateRAB(robotPos, desPos):
	rho = np.linalg.norm(desPos- robotPos[0:2])
	alp = -robotPos[2] + np.arctan2(desPos[1]-robotPos[1], desPos[0]-robotPos[0])
	bta = -robotPos[2] - alp
	return np.array([[rho],[alp],[bta]])


"""Calculate the next X, Y and Theta using the previous XYT and the current RAB"""
def calculateXYT(robotPos, currentRAB):
	x_nxt = robotPos[0] + kr * currentRAB[0] * math.cos(robotPos[2])
	y_nxt = robotPos[1] + kr * currentRAB[0] * math.sin(robotPos[2])
	t_nxt = robotPos[2] + (ka * currentRAB[1] +kb*currentRAB[2])
	return np.array([[x_nxt],[y_nxt],[t_nxt]])




# -----Input from the user-----
print ("Simple Controller for a Robot ")

# Ask for control constants
kr = 0.18	# kr > 0
#kr = float(input("Insert the value of kr? (It has to be greater than 0): "))# kr > 0
ka = 0.2	# (ka-kr) > 0	# ka must be greater than kr
#ka = float(input("Insert the value of ka? (It has to be greater than kr): "))	# (ka-kr) > 0	# ka must be greater than kr
kb = -0.05	# kb < 0
#kb = float(input("Insert the value of kb? (It has to be less than 0): "))	# kb < 0	# ka must be greater than kr


#TODO: ask for simulation time
t = 100

# TODO:choose trajectory

# ----- Calculate cartesian trajectory in time
# --   Trajectories
# - Linear
# the trajectory is composed of a linear trajectory a 45 deg from the originwith a length of 10 units
v = 10 / t
tjt = np.zeros((2,t))

#for x in range(t):
#	# x(t) = x0 + v * t
#	# y(t) = x(t) = y0 + v * t
#	tjt[:,x] = x


# Circular
#xCenter = t/4
#yCenter = 0
#radius = t/4

#for x in range(0,int(xCenter + radius)):
#	tjt[0,x] = x
#	tjt[1,x] = math.sqrt(abs(radius**2-(tjt[0,x]-xCenter)**2))+yCenter

#for x in range(int(xCenter + radius), t):
#	tjt[0,x] = 2 * (xCenter + radius) - x
#	tjt[1,x] = -math.sqrt(abs(radius ** 2 - (tjt[0,x] - xCenter) ** 2)) + yCenter


# - TODO: Semisinusoid
xCenter = t/8
yCenter = 0
radius = t/8

for x in range(int(t/4)):
	tjt[0,x] =x
	#(x−a)2 + (y−b)2 = r2
	tjt[1,x] =math.sqrt(abs((tjt[0,x]-xCenter) ** 2 - radius**2)) + yCenter

xCenter = t / 8 + t / 4
for x in range(int(t/4),int(t/2)):
	tjt[0,x] =x
	#(x−a)2 + (y−b)2 = r2
	tjt[1,x] =-math.sqrt(abs((tjt[0,x]-xCenter) ** 2 - radius**2)) + yCenter

for x in range(int(t/2),int(t/2+t/4)):
	tjt[0,x] = t - x
	#(x−a)2 + (y−b)2 = r2
	tjt[1,x] =math.sqrt(abs((tjt[0,x]-xCenter) ** 2 - radius**2)) + yCenter

xCenter = t/8
for x in range(int(t/2+t/4),t):
	tjt[0,x] = t - x
	#(x−a)2 + (y−b)2 = r2
	tjt[1,x] =-math.sqrt(abs((tjt[0,x]-xCenter) ** 2 - radius**2)) + yCenter
# - TODO: Square
# - TODO: Square rounded corners

# ----- Set Initial values for robot positions
rp = np.array([[0],[0], [0]])

# ----- Calculate the initial rho, alpha, beta
rab = calculateRAB(rp[:,0],tjt[:,0])
# ----- for loopfor the whole time

for x in range(1, t):
	#Calculate next x, y and theta
	x_nxt = rp[0,-1] + kr * rab[0,-1] * math.cos(rp[2,-1])
	y_nxt = rp[1,-1] + kr * rab[0,-1] * math.sin(rp[2,-1])
	t_nxt = rp[2,-1] + (ka * rab[1,-1] +kb*rab[2,-1])
	
	# Calculate next RAB based on the previous robot position and the desired position
	rho_nxt = np.linalg.norm(tjt[:,x]- rp[:-1,-1])
	apa_nxt = -rp[2,-1] + np.arctan2(tjt[1,x ] - rp[1,-1], tjt[0,x ] - rp[0,-1])
	bta_nxt = -rp[2,-1] - rab[1,-1]

	#store the values
	rab = np.hstack((rab,np.array([[rho_nxt],[apa_nxt],[bta_nxt]])))
	rp = np.hstack((rp,np.array([[x_nxt],[y_nxt],[t_nxt]])))

# ----- Plot position error VS time

posError = tjt - rp[0:2,:]	#Position error
xAxis = np.arange(0,t)	#list for the time

## -- X axis error
ax1= plt.subplot(2, 1, 1)
plt.title("X axis error") 
plt.xlabel("time") 
plt.ylabel("error") 
plt.plot(xAxis,posError[0,:]) 

## -- Y axis error
ax2= plt.subplot(2, 1, 2)
plt.title("Y axis error") 
plt.xlabel("time") 
plt.ylabel("error") 
plt.plot(xAxis,posError[1,:]) 

plt.draw() 


## ----- Robot movement across the trajectory

#animation
fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(15, 15)

#ax_anim = plt.axes(xlim=(0, t), ylim=(0, t))	#linear
ax_anim = plt.axes(xlim=(-10, t), ylim=(-t,t))	#semisinusoid

plt.title("Robot movement trying to follow the path") 

planned = plt.plot(tjt[0,:], tjt[1,:],'g-')
real = plt.plot(rp[0,:], rp[1,:], 'r-.')


def init():
    patch = plt.Arrow(0,0,0,0)
    ax_anim.add_patch(patch)
    return patch,

def animate(i):
	patch = plt.Arrow(rp[0,i],rp[1,i],(t/20)*math.cos(rp[2,i]),(t/20)*math.sin(rp[2,i]), width=(t/30))
	ax_anim.add_patch(patch)
	return patch,

anim = animation.FuncAnimation(fig, animate, 
                               init_func=init, 
                               frames=t, 
                               interval=200,
                               blit=True)

plt.show()

