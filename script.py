#   Teknologiprosjekt øving 1: Robotarium experiment with Motion planning and Control
#   Student: Mathias Mellemstuen
#   Python interpreter: 3.8.2 64 bit

import rps.robotarium as robotarium
import time
import numpy as np
import math

running = True
numberOfRobots = 1
lineWidth = 1
initialConditions = np.array(np.mat('0;0.00;3.1415'))
r = robotarium.Robotarium(number_of_robots=numberOfRobots, show_figure=True, initial_conditions=initialConditions,sim_in_real_time=True)


def normalize(min, max, value): 
    return min + (max - min) * value 

def createCircle(x, y, fromRadians, toRadians, step, normalizeMin, normalizeMax): 
    current = fromRadians
    circle = [[],[]]
    while(current < toRadians):
        circle[0].append(x + normalize(normalizeMin, normalizeMax, math.cos(current)))
        circle[1].append(y + normalize(normalizeMin, normalizeMax, math.sin(current)))
        current += step

    return circle


def createFullPath(): 
    global path

    path = [
        [0.0, 0.2, 1.0],
        [0.0,-0.2,-0.2]
    ]
    c = createCircle(path[0][-1], path[1][-1] + 0.2, math.pi + (math.pi / 2.0), (2 * math.pi) + math.pi / 2.0, 0.1, 0, 0.2)
    path[0].extend(c[0])
    path[1].extend(c[1])
    path[0].append(0.2)
    path[1].append(0.2)
    path[0].append(-0.2)
    path[1].append(-0.2)
    path[0].append(-1.0)
    path[1].append(-0.2)
    c2 = createCircle(path[0][-1], path[1][-1] + 0.2, (math.pi / 2.0), math.pi + (math.pi / 2.0), 0.1, 0, 0.2)
    path[0].extend(reversed(c2[0]))
    path[1].extend(reversed(c2[1]))
    path[0].append(-0.2)
    path[1].append(0.2)
    path[0].append(0)
    path[1].append(0)

def plotTrack():
    r.axes.plot(path[0],path[1],color='red',linewidth = lineWidth)

# setVelocity sets the linear and angular velocity of the robot
def setVelocity(linear, angular): 
    global velocity
    
    linearSpeed = float(linear)
    angularSpeed = float(angular) 
    velocity = np.array([linearSpeed, angularSpeed])

i = 0
def calculateNextVelocity():
    if(i + 1 > path[0].__len__

# Function is getting looped as long as running = True
def robotLogic(): 
    position = r.get_poses()
    calculateNextVelocity()
    v = velocity
    v.shape = (2,1)
    r.set_velocities(numberOfRobots, v)
    r.step()


# Creating the full path and assigning it to 'path'
createFullPath()

# Plotting the track/path background
plotTrack()

# Initially sets the velocity for defining 'global velocity', not doing this will throw an error. 
setVelocity(0.0,0.0)

# Looping the robot logic
while running: robotLogic()

# Calling this function at the end of the script to display potential errors.
r.call_at_scripts_end()