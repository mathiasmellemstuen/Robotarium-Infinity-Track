#   Teknologiprojekt øving 1: Robotarium experiment with Motion planning and Control
#   Student: Mathias Mellemstuen
#   Python interpreter: 3.8.2 64 bit

import rps.robotarium as robotarium
import time
import numpy as np

running = True
numberOfRobots = 1
initialConditions = np.array(np.mat('0;0.00;3.1415'))
r = robotarium.Robotarium(number_of_robots=numberOfRobots, show_figure=True, initial_conditions=initialConditions,sim_in_real_time=True)

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

# Initially sets the velocity for defining 'global velocity', not doing this will throw an error. 
setVelocity(-1,0)

# Looping the robot logic
while running: robotLogic()

r.call_at_scripts_end()