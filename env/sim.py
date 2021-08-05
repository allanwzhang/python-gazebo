import subprocess
import time
import sys
import socket
import numpy as np
from Controllers.proto.vector8d_pb2 import Vector8d
import controller


"""

sim.py

Initiates the simulation for the Tarot T18 Octoctopter in Gazebo.
Uses controller.py to create the base environment and controllers for the Octorotor.
To modify gains, see controller.py.


"""


def start_sim():
	# Start the Simulator
	p = subprocess.Popen(["gazebo", "../gazebo/worlds/sim_tarot.world"], shell=False)
	print("Starting gzserver with process ID=", p.pid)
	#p = subprocess.Popen(["gzclient"], shell=False)
	time.sleep(7)
	
	

#start_sim()
# Create environment with initial state
# Version specifies which Control Allocation to use:
#	Define version = 0 for DJI-S1000.
#	Define version = 1 for Tarot T18 (default)
print("Creating Environment.")
env = controller.Create(version=1)



if __name__ == '__main__':
	
	
	print("Entering motor Simulation Loop.")
	xarr = [ 0, 1, 4, -2]
	yarr = [ 0, 2, 6, 4]
	altitude = 3
	env.step(xarr, yarr, altitude, velocity=1)
	
	"""
	print("Starting Simulation Demo.")
	xarr = [92]
	yarr = [118]
	altitude = 18
	env.step(xarr, yarr, altitude, velocity=2)
	"""
	
	
	print("Completed")
	
	
	
	
	
	




	

	
	
