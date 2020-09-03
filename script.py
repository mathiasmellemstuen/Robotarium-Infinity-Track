#   Teknologiprosjekt øving 1: Robotarium experiment with Motion planning and Control
#   Student: Mathias Mellemstuen
#   Python interpreter: 3.8.2 64 bit

import rps.robotarium as robotarium
import time
import numpy as np

running = True
numberOfRobots = 1
lineWidth = 1
initialConditions = np.array(np.mat('0;0.00;3.1415'))
r = robotarium.Robotarium(number_of_robots=numberOfRobots, show_figure=True, initial_conditions=initialConditions,sim_in_real_time=True)
pathQuarter = [
    [0,0,0.9],
    [0,0.9,0.9]
]
def createFullPath(): 
    global path
    path = pathQuarter


def plotTrack():
    r.axes.plot(path[0],path[1],color='red',linewidth = lineWidth)

# setVelocity sets the linear and angular velocity of the robot
def setVelocity(linear, angular): 
    global velocity
    
    linearSpeed = float(linear)
    angularSpeed = float(angular) 
    velocity = np.array([linearSpeed, angularSpeed])
    velocity.shape = (2,1)

# Funcion is getting looped as long as running = True
def robotLogic(): 
    position = r.get_poses()
    r.set_velocities(numberOfRobots, velocity)
    r.step()



# Creating the full path and assigning it to 'path'
createFullPath()

# Plotting the track/path background
plotTrack()

# Initially sets the velocity for defining 'global velocity', not doing this will throw an error. 
setVelocity(0,0)

# Looping the robot logic
while running: robotLogic()

# Calling this function at the end of the script to display potential errors.
r.call_at_scripts_end()