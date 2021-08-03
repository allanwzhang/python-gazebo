import subprocess
import time
import sys
import socket
import numpy as np

from gymfc.msgs.packet_pb2 import Packet
from gymfc.msgs.publish_pb2 import Publish
from google.protobuf import descriptor
from gymfc.msgs import vector3d_pb2
from gymfc.msgs.vector3d_pb2 import Vector3d
from gymfc.msgs import vector8d_pb2
from gymfc.msgs.vector8d_pb2 import Vector8d

import controller


"""

sim.py

Initiates the simulation for the Tarot T18 Octoctopter in Gazebo and sets up 
the interface to publish messages for commanding the motor speeds of the 8 motors.


"""

# Following variables are for publishing on the Gazebo Vector3d protobuff topic
MASTER_TCP_IP   = '127.0.0.1'
MASTER_TCP_PORT = 11345


# Variables for state data received from Gazebo
position = [0, 0, 0]
lin_vel = [0, 0, 0]
attitude = [0, 0, 0]
ang_vel = [0, 0, 0]


def start_sim():
	# Start the Simulator
	p = subprocess.Popen(["gazebo", "../gazebo/worlds/sim_env.world"], shell=False)
	print("Starting gzserver with process ID=", p.pid)
	#p = subprocess.Popen(["gzclient"], shell=False)
	time.sleep(7)
	
	

#start_sim()

# Create environment with initial state
print("Creating Environment.")
env = controller.Create()


def print_state():
	print("Position: \t",
		round(position[0], 2), "\t\t", round(position[1], 2), "\t\t", round(position[2], 2))
	print("Linear Vel: \t",
		round(lin_vel[0], 2), "\t\t", round(lin_vel[1], 2), "\t\t", round(lin_vel[2], 2))
	print("Attitude: \t",
		round(attitude[0], 2), "\t\t", round(attitude[1], 2), "\t\t", round(attitude[2], 2))
	print("Angular Vel: \t",
		round(ang_vel[0], 2), "\t\t", round(ang_vel[1], 2), "\t\t", round(ang_vel[2], 2))



# -------------------------------------
#		Simulate Time
# Method to simulate a voltage input for a specific duration of time. 
# Arguments: Voltage_array and sim_steps
#		* Voltage_array: 8 Dimensional array of voltages to apply to 8 individual motors
#		* sim_steps: number of iterations to make with the current voltage setting.
# -------------------------------------
def sim_time(voltage_array, sim_steps):
	
	update_num = 0
	
	while True:
		if (update_num == sim_steps):
			break
		
		ang_vel_1 = motor_1.update(voltage_array[0], 0.001)
		ang_vel_2 = motor_2.update(voltage_array[1], 0.001)
		ang_vel_3 = motor_3.update(voltage_array[2], 0.001)
		ang_vel_4 = motor_4.update(voltage_array[3], 0.001)
		ang_vel_5 = motor_5.update(voltage_array[4], 0.001)
		ang_vel_6 = motor_6.update(voltage_array[5], 0.001)
		ang_vel_7 = motor_7.update(voltage_array[6], 0.001)
		ang_vel_8 = motor_8.update(voltage_array[7], 0.001)
		
		
		rpm1 = round(ang_vel_1[0] * 9.5493)
		rpm2 = round(ang_vel_2[0] * 9.5493)
		rpm3 = round(ang_vel_3[0] * 9.5493)
		rpm4 = round(ang_vel_4[0] * 9.5493)
		rpm5 = round(ang_vel_5[0] * 9.5493)
		rpm6 = round(ang_vel_6[0] * 9.5493)
		rpm7 = round(ang_vel_7[0] * 9.5493)
		rpm8 = round(ang_vel_8[0] * 9.5493)
		
		#print("\tSending RPM array to /tarot/motors.")
		msg_array = Vector8d()
		msg_array.motor_1 = rpm1
		msg_array.motor_2 = rpm2
		msg_array.motor_3 = rpm3
		msg_array.motor_4 = rpm4
		msg_array.motor_5 = rpm5
		msg_array.motor_6 = rpm6
		msg_array.motor_7 = rpm7
		msg_array.motor_8 = rpm8
		
		state, _ = publisher.loop.run_until_complete(publisher.ac_protocol.write(msg_array))
		
		# Now Update State values from Gazebo
		position[0], position[1], position[2] = state[0], state[1], state[2]
		lin_vel[0], lin_vel[1], lin_vel[2] = state[3], state[4], state[5]
		attitude[0], attitude[1], attitude[2] = state[6], state[7], state[8]
		ang_vel[0], ang_vel[1], ang_vel[2] = state[9], state[10], state[11]
		
		#time.sleep(0.0001)
		update_num += 1



if __name__ == '__main__':
	

	#Start the Simulator
	#pid = start_sim()
	#print("Simulator started")
	#time.sleep(8)
	
	
	print("Entering motor Simulation Loop.")
	xarr = [ 0, 1, 4, 4]
	yarr = [ 0, 0, 2, 4]
	altitude = 3
	env.step(xarr, yarr, altitude, 6000)
	
	"""
	print("Starting Simulation Demo.")
	altitude = 19
	#xarr = [18.8951, 112.28, 112.28, 112.28, 112.28]
	#yarr = [-23.9868, 120.826, 120.826, 120.826, 120.826]
	
	xarr = [112.28, 112.28, 112.28, 112.28]
	yarr = [120.826, 120.826, 120.826, 120.826]
	env.step(xarr, yarr, altitude, 2000)
	
	
	xarr = [112.28, 112.28]
	yarr = [120.826, 120.826]
	altitude = 8
	env.step(xarr, yarr, altitude, 10000)
	"""
	
	print("Completed")
	
	
	
	
	
	




	

	
	
