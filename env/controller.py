import gym
import time
from Controllers.octorotor_base_env import OctorotorBaseEnv
from Controllers.MotorController import MotorController
from Controllers.AttitudeController import AttitudeController
from Controllers.PositionController import PositionController
from Controllers.AltitudeController import AltitudeController
from motor import Motor

import numpy as np
import sys
import math
import os

# Simulation Parameters

g = 9.81		# gravity
m0 = 10.66		# mass of octocoptor
d = 1.8503e-06	# Drag constant. From MATLAB model
b = 9.8419e-05	# Rotor thrust constant - from MATLAB model
Ixx = 0.2506	# X inertia
Iyy = 0.2506	# Y inertia
Izz = 0.4538	# Z inertia
J = np.array([[Ixx, 0, 0], [0, Iyy, 0], [0, 0, Izz]])	# Inertial Matrix

OctorotorParams = {
	"g": g,
	"m0": 10.66,
	"d": d,
	"b": b,
	"l": 0.635,
	"omegamax": 670,	# max rotational velocity in rad/s
	"Ixx": Ixx,
	"Iyy": Iyy,
	"Izz": Izz,
	"dt": 0.001,
}


MotorParams = {
        "Izzm": 5E-5,
        "km": 0.0265,		# Ke and Km should be the same value
        "ke": 0.0265,
        "R": 0.2799,		# R - resistance
        "d": d,				# d - drag constant
        "komega": 1,		# komega -proportional motor gain
        "maxv": 26.1
}


PositionParams = {
        "kpx": 0.2,		# proportional gain x
        "kdx": 0.1,		# derivative gain x
        "kpy": 0.2,		# proportional gain y
        "kdy": 0.1,		# derivative gain y
        "min_angle": -12*math.pi/180,		# min angle is -12 degrees
        "max_angle": 12*math.pi/180			# max angle is +12 degrees
}


AttitudeParams = {
        "kd": 10,		# derivative attitude gain
        "kp": 50,		# Proprotional attitude gain
        "j" : J
}


AltitudeParams = {
        "g": g,
        "m0": m0,
        "kdz": 48,	# Derivative Thrust gain
        "kpz": 72	# Proportional Thrust gain
}



def Create(version=1):
	
	#print("Things imported.")
	
	# Setup PID Controllers
	altc = AltitudeController(AltitudeParams)
	attc = AttitudeController(AttitudeParams)
	posc = PositionController(PositionParams)
	
	# Create Motor and motor controllers
	motor = Motor()
	motorc = MotorController(MotorParams)
	OctorotorParams["motor"] = motor
	OctorotorParams["motorController"] = motorc
	OctorotorParams["positionController"] = posc
	OctorotorParams["attitudeController"] = attc
	OctorotorParams["altitudeController"] = altc
	OctorotorParams["total_step_count"] = 5000
	OctorotorParams["reward_discount"] = 1
	resistance = np.full(8, 0.2799)
	OctorotorParams["resistance"] = resistance
	
	env = OctorotorBaseEnv(OctorotorParams, version)
	
	return env
	
	
	#_, rew, done, err = env.step([0.2, 0.1, 0.2, 0.1]
	
	
	
	




