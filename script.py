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
initialConditions = np.array(np.mat('-1;0.00;0'))
currentTarget = 0
acceptableDistanceToTarget = 0.05

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


def plotLine(x1, y1, x2, y2):
    r.axes.plot([x1, x2],[y1, y2],color='blue',linewidth = 1)

def plotTrack():
    r.axes.plot(path[0],path[1],color='red',linewidth = lineWidth)

# setVelocity sets the linear and angular velocity of the robot
def setVelocity(linear, angular): 
    global velocity
    
    linearSpeed = float(linear)
    angularSpeed = float(angular) 
    velocity = np.array([linearSpeed, angularSpeed])


def calculateVectorLength(x1,y1,x2,y2):
    return math.sqrt(math.pow(x2 - x1,2) + math.pow(y2 - y1, 2))

def calculateAngleBetweenPoints(x1,y1,x2,y2):
    
    if x1 is x2 and y1 is y2: return 0 # Checking if the points are at the same coordinate, returning 0 if that's the case. 

    vec1Length = calculateVectorLength(x1, y1, x2, y2)
    vec2Length = calculateVectorLength(x1, y1, x2, y1) # This vector will always point either towards 0 rad or pi rad (enhetssirkel), it will always be a straight line. That's why both y parameters are y1.
    
    if y2 <= y1:
        if x2 < x1:
            return -math.pi + math.acos(vec2Length / vec1Length)
        if x2 >= x1:
            return -math.acos(vec2Length / vec1Length)
    
    if(y2 > y1):
        if(x2 < x1):
            return (math.pi * 0.5) + math.acos(vec2Length / vec1Length)
        if(x2 >= x1):
            return math.acos(vec2Length / vec1Length)
        
def calculateNextVelocity(position):
    
    global currentTarget

    targetX = path[0][currentTarget]
    targetY = path[1][currentTarget]

    distanceToTarget = calculateVectorLength(targetX,targetY,position[0],position[1])    
    
    currentTarget = currentTarget if not (distanceToTarget < acceptableDistanceToTarget) else (currentTarget + 1) if (currentTarget is not len(path[0]) - 1) else 0

    targetX = path[0][currentTarget]
    targetY = path[1][currentTarget]

    targetAngle = calculateAngleBetweenPoints(position[0], position[1], targetX, targetY)
    errorAngle = position[2] - targetAngle;
    setVelocity(0.05, errorAngle)
    print(targetAngle)
    
    plotLine(position[0], position[1], targetX, targetY)

# Function is getting looped as long as running = True
def robotLogic():
    position = r.get_poses()
    calculateNextVelocity(position)
    v = velocity
    v.shape = (2,1)
    r.set_velocities(numberOfRobots, v)
    r.step()



# Creating the full path and assigning it to 'path'
createFullPath()



print("Angle in radians: ")
print(calculateAngleBetweenPoints(0.0, 0.0, 0.0, 0.0))

# Plotting the track/path background
plotTrack()

# Initially sets the velocity for defining 'global velocity', not doing this will throw an error. 
setVelocity(0.2, 0)

# Looping the robot logic
while running: robotLogic()

# Calling this function at the end of the script to display potential errors.
r.call_at_scripts_end()