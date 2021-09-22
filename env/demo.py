from posixpath import abspath
import subprocess
import time
from datetime import datetime
import os
from pathlib import Path
import socket
import psutil
import numpy as np
import asyncio
from Controllers.proto.vector8d_pb2 import Vector8d
from Controllers import GazeboPublisher
from motor import Motor
import controller
import pandas as pd




def start_sim():
	# Start the Simulator
	p = subprocess.Popen(["gazebo", "demo.world"], shell=False)
	print("Starting gzserver with process ID=", p.pid)
	#p = subprocess.Popen(["gzclient"], shell=False)
	time.sleep(7)



# Following variables are for publishing on the Gazebo Vector3d protobuff topic
MASTER_TCP_IP   = '127.0.0.1'
MASTER_TCP_PORT = 11345

#start_sim()

# Create UDP Publisher to communicate with Tarot_Env_Plugin
publisher = GazeboPublisher.GazeboPublisher(MASTER_TCP_IP, MASTER_TCP_PORT)

# Send Command to simply step the iteration to return an initial state
initialize = Vector8d()
initialize.motor_1 = 0
initialize.motor_2 = 0
initialize.motor_3 = 0
initialize.motor_4 = 0
initialize.motor_5 = 0
initialize.motor_6 = 0
initialize.motor_7 = 0
initialize.motor_8 = 0

# Run message twice to update state after initial dummy state message
state, _ = publisher.loop.run_until_complete(publisher.ac_protocol.write(initialize))
state, _ = publisher.loop.run_until_complete(publisher.ac_protocol.write(initialize))

global stepCount
stepCount = 2
	
# Initialize motor objects
motor_1 = Motor()
motor_2 = Motor()
motor_3 = Motor()
motor_4 = Motor()
motor_5 = Motor()
motor_6 = Motor()
motor_7 = Motor()
motor_8 = Motor()



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
		
		#time.sleep(0.05)
		update_num += 1


if __name__ == "__main__":

	abs_path = os.path.abspath(os.path.dirname(__file__))
	
	print("Starting Flight Demo.")
	
	print("Reading in flight data.")
	my_sheet = 'Sheet1'
	paths_dir = Path(__file__).resolve().parent / 'paths'
	file_name = paths_dir / 'flight_data.xlsx'
	df = pd.read_excel(file_name, sheet_name = my_sheet, engine='openpyxl')
	
	sim_end = 0
	if file_name.name == 'fault.xlsx':
		sim_end = 5900
	else:
		sim_end = 50000
	
	print("taking off to stable atlitude")
	msg_array = np.array([13, 13, 13, 13, 13, 13, 13, 13])
	sim_time(msg_array, 3000)
	
	#s = input("Press [ENTER] to iterate next maneuver.")
	print("Decelerate.")
	decelerate = np.array([9, 9, 9, 9, 9, 9, 9, 9])
	sim_time(decelerate, 1000)
	print("Hover.")
	hover_array = np.array([11.68, 11.68, 11.68, 11.68, 11.68, 11.68, 11.68, 11.68])
	sim_time(hover_array, 2000)
	
	
	i = 0
	print("Beginning Trajectory Demo.")
	for index, row in df.iterrows():
		
		if (stepCount >= sim_end):
			break
				
		i = 0
		msg_array = Vector8d()
		msg_array.motor_1 = row[0]
		msg_array.motor_2 = row[1]
		msg_array.motor_3 = row[2]
		msg_array.motor_4 = row[3]
		msg_array.motor_5 = row[4]
		msg_array.motor_6 = row[5]
		msg_array.motor_7 = row[6]
		msg_array.motor_8 = row[7]
		while i < 10:
			publisher.loop.run_until_complete(publisher.ac_protocol.write(msg_array))
			stepCount += 1
			#time.sleep(0.05)
			i += 1
		#print("Location: ( " , state[0] , " , " , state[1] , " , " , state[2] , " ).")
		
	
	print("Trajectory Route Completed.")
	
	if file_name == 'paths/fault.xlsx':
		print("Crashing.")
		sim_time(np.array([13, 13, 13, 13, 13, 13, 13, 13]), 200)
		sim_time(np.array([6, 6, 6, 6, 6, 6, 6, 6]), 2000)
		sim_time(np.array([0, 0, 0, 0, 0, 0, 0, 0]), 3000)
	
	
	#TODO: Landing function below doesn't work
	if file_name == 'paths/flight_data.xlsx':
		print("Landing.")
		
		#sim_time(hover_array, 3000)
		sim_time(np.array([11.5, 11.5, 11.5, 11.5, 11.5, 11.5, 11.5, 11.5]), 10000)
		#sim_time(hover_array, 1000)
		sim_time(np.array([11.4, 11.4, 11.4, 11.4, 11.4, 11.4, 11.4, 11.4]), 10000)
	
	
	
	
	
