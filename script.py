#   Teknologiprosjekt øving 1: Robotarium experiment with Motion planning and Control
#   Student: Mathias Mellemstuen
#   Python interpreter: 3.8.2 64 bit

import rps.robotarium as robotarium
import time
import numpy as np
import math
import random

startingX = random.uniform(-0.5, 0.5)
startingY = random.uniform(-0.5, 0.5)
startingAngle = random.uniform(-math.pi, math.pi)
initialConditions = np.array(np.mat(str(startingX) +';' + str(startingY) + ';' + str(startingAngle)))
running = True # Script is running as long as this is true. 
numberOfRobots = 1 # Code needs modification for handling more than one robot. 
lineWidth = 1 # The width of the red line on the track.
currentTarget = 0 # The current target/waypoint
acceptableDistanceToTarget = 0.05 # The robot is changing to the next waypoint if it is within this value to the current waypoint.
acceptableAngleError = 0.1 # In radians
maxSpeed = 0.3 # The bot's max speed.
turningSpeed = 1 # The bot's max angular speed. 
brakeDistance = 0.3 # Distance before the robot is beginning to break.
acceptableAngelErrorAtBrakeDistance = 0.1
verbose = True # Prints to terminal if this is true. 

r = robotarium.Robotarium(number_of_robots=numberOfRobots, show_figure=True, initial_conditions=initialConditions,sim_in_real_time=True)

# Normalizing a value between min - max
def normalize(min, max, value): 
    return min + (max - min) * value 

# Creating a part of a circle fromRadians -> toRadians
# x, y: starting position of the circle
# step: Radians until the next point on the circle is added. A small value gives great precision
# normalizeMin, normalizeMax: Deciding the size of the circle with normalizing the value. 
# Returning: Two dimentional array with the points (x,y values)
def createCircle(x, y, fromRadians, toRadians, step, normalizeMin, normalizeMax): 
    current = fromRadians
    circle = [[],[]]
    while(current < toRadians):
        circle[0].append(x + normalize(normalizeMin, normalizeMax, math.cos(current)))
        circle[1].append(y + normalize(normalizeMin, normalizeMax, math.sin(current)))
        current += step

    return circle

# Creating the inifinity track and assigning 'path' variable to it. 
def createInfinityPath(): 
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

# Plotting a red line between the points in the path. 
def plotTrack():
    r.axes.plot(path[0],path[1],color='red',linewidth = lineWidth)

# setVelocity sets the linear and angular velocity of the robot
def setVelocity(linear, angular): 
    global velocity
    linearSpeed = float(linear)
    angularSpeed = float(angular) 
    velocity = np.array([linearSpeed, angularSpeed])

    #For debugging:
    if(verbose):
        print("Linear velocity:", linear)
        print("Angular velocity:", angular)

# Calculates and returning the shortest distance between two points in a two dimentional space utilizing pythagoras sentence. 
def calculateDistanceBetweenTwoPoints(x1,y1,x2,y2):
    return math.sqrt(math.pow(x2 - x1,2) + math.pow(y2 - y1, 2))

# Returning the shortest direction to rotate to meet the target angle. Returns either 1/-1. Does also return 0 when the angles are the same. 
def shortestWayBetweenTwoAngles(angle1, angle2): 
    angle1 = float(angle1)
    angle2 = float(angle2)

    if angle1 < angle2 and angle2 - angle1 <= math.pi:
        return 1.0
    if angle1 < angle2 and angle2 - angle1 > math.pi:
        return -1.0
    if angle1 > angle2 and angle1 - angle2 <= math.pi:
        return -1.0
    if angle1 > angle2 and angle1 - angle2 > math.pi:
        return 1.0

    return 0.0

# Calculate the angle between two points in radians.
def calculateAngleBetweenPoints(x1,y1,x2,y2):
    deltaX = x2 - x1
    deltaY = y2 - y1
    return math.atan2(deltaY, deltaX)

# Converting radians at the form of 0 -> pi -> -pi -> -0 to 0 -> 2pi
def convertAngle0To2Pi(a):
    return a if a >= 0.0 else 2 * math.pi + a

# Returning the shortest difference between two angles
def calculateAngleDifference(angle1, angle2):
    return math.pi - abs(abs(angle1 - angle2) - math.pi)

# Robot slow down / speed up based on the angle and distance to the next target. 
def calculateSpeedModifierBasedOnAngleDifferenceAndDistance(angle, targetAngle, distanceToTarget):
    difference = float(calculateAngleDifference(angle, targetAngle))
    angleDistErrModifier = acceptableAngelErrorAtBrakeDistance / difference if distanceToTarget < brakeDistance and acceptableAngelErrorAtBrakeDistance < difference else 1
    
    return (maxSpeed * (difference / ((2.0*math.pi)))) / angleDistErrModifier

# Calculating and setting both angular and linear velocity. 
def calculateNextVelocity(position):
    
    global currentTarget

    targetX = path[0][currentTarget]
    targetY = path[1][currentTarget]

    distanceToTarget = calculateDistanceBetweenTwoPoints(targetX,targetY,position[0],position[1])    
    
    currentTarget = currentTarget if not (distanceToTarget < acceptableDistanceToTarget) else (currentTarget + 1) if (currentTarget is not len(path[0]) - 1) else 0

    targetX = path[0][currentTarget]
    targetY = path[1][currentTarget]

    targetAngle = convertAngle0To2Pi(calculateAngleBetweenPoints(position[0], position[1], targetX, targetY))
    angle = convertAngle0To2Pi(position[2])

    speed = maxSpeed - calculateSpeedModifierBasedOnAngleDifferenceAndDistance(angle, targetAngle, distanceToTarget)
    if speed < 0: speed = 0
    
    setVelocity(speed,0 if calculateAngleDifference(angle, targetAngle) < acceptableAngleError else turningSpeed * shortestWayBetweenTwoAngles(angle, targetAngle))

# Function is getting looped as long as running = True
def robotLogic():
    position = r.get_poses()
    calculateNextVelocity(position)
    v = velocity
    v.shape = (2,1)
    r.set_velocities(numberOfRobots, v)
    r.step()



# Creating the full path and assigning it to 'path'
createInfinityPath()

# Plotting the track/path background
plotTrack()

# Looping the robot logic
while running: robotLogic()

# Calling this function at the end of the script to display potential errors.
r.call_at_scripts_end()