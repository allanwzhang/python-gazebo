import numpy as np
import scipy.integrate
import math
from numba import jit
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints

import socket
import time
import codecs
import pygazebo
import asyncio
from gymfc.msgs import vector8d_pb2
from gymfc.msgs.vector8d_pb2 import Vector8d



"""
@jit 
def _calc_rotation_matrix(angles):
    ct = math.cos(angles[0])
    cp = math.cos(angles[1])
    cg = math.cos(angles[2])
    st = math.sin(angles[0])
    sp = math.sin(angles[1])
    sg = math.sin(angles[2])
    R = np.array([[cg*cp, cp*cg*st-sg*ct, sp*cg*ct+sg*st], [cp*sg, sg*cp*st+cg*ct, sg*cp*ct-st*sg], [-sp, cp*st, cp*ct]])
    return R


@jit
def _calc_rotation_inverse_a(angle):
    phi = angle[0]
    theta = angle[1]
    psi = angle[2]
    sphi = math.sin(phi)
    tt = math.tan(theta)
    nphi = math.cos(phi)
    ctheta = math.cos(theta)
    r = np.array([[1, sphi*tt, nphi*tt], [0, nphi, -sphi], [0, sphi/ctheta, nphi/ctheta]], dtype="float32")
    return r
"""

class ActionProtocol:
    
    def __init__(self):
        self.state_message = None
        self.packet_received = False
        self.exception = None
        self.send_time = None
        self.timeout = 30
        self.first = True
        print("\tWriter Created.")
    
    
    def connection_made(self, transport):
        self.transport = transport
    
    
    async def write(self, motor_vector, world_control=0):
        """ Write the motor values to the ESC and then return 
        the current sensor values and an exception if anything bad happend.
        
        Args:
            motor_vector: Vector8d object of motor values
        """
        self.packet_received = False
        
        # If statement to force UDP message to continue after first message.
        # For some reason, python publisher does not receive the first message and gets stuck
        # So, don't worry about the first message, since it will be updated the next 0.001 second.
        if self.first == True:
        	self.packet_received = True
        	self.state_message = "-1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1"
        	self.first = False
        
        self.send_time = time.time()
        self.transport.sendto(ActionPacket(motor_vector).encode())

        # Pass the exception back if anything bad happens
        while not self.packet_received:
            if self.exception:
                return (None, self.exception)
            elif time.time() > self.send_time + self.timeout:
                return (None, ReadTimeoutException("Timeout reached waiting for response."))
            await asyncio.sleep(0.001)
        
        return (self.state_message, None)
    
    def error_received(self, exc):
        self.exception = exc
    
    def datagram_received(self, data, addr):
        """ Receive a UDP datagram
        Args:
            data (bytes): raw bytes of packet payload 
            addr (string): address of the sender
        """
        # Everything is OK, reset
        self.exception = None
        self.packet_received = True
        self.state_message = StatePacket().decode(data)
    
    def connection_lost(self, exc):
        print("Socket closed, stop the event loop")
        loop = asyncio.get_event_loop()
        loop.stop()
        

class ActionPacket:
    def __init__(self, motor_vector, world_control=0):
        """
        Args:
            motor (np.array): an array of motor control signals.   
        """
        self.ac = motor_vector

    def encode(self):
        """  Encode packet data"""
        msg = self.ac.SerializeToString() 
        #print ("Sending ", self.ac, " to Gazebo size=", len(msg), " ",   msg.hex())
        return msg

    def __str__(self):
        return str(self.ac)
    

class StatePacket:
    
    def decode(self, data):
       state_string = str(data, 'utf-8')
       state_string_array = state_string.split()
       state = [float(numeric_string) for numeric_string in state_string_array]
       return state  






class Octocopter:
    def __init__(self, OctorotorParams, stepNum = 0, T=0, tau=np.array([0,0,0])):
        self.prevx = [0, 0]
        self.xfilter = []
        self.yfilter = []
        self.stepNum = stepNum
        self.u = np.zeros(6)

        # State vector looks like
        # Pos (x, y, z) , Vel (x, y, z), Orient (Phi, Theta, Psi), AngVel (Phi, Theta, Psi)
        self.state = np.zeros(12, dtype = np.float32)
        self.T = np.float32(T)
        self.tau = np.float32(tau)
        self.m0 = np.float32(OctorotorParams["m0"])
        self.g = np.float32(OctorotorParams["g"])
        self.Ixx = np.float32(OctorotorParams["Ixx"])
        self.Iyy = np.float32(OctorotorParams["Iyy"])
        self.Izz = np.float32(OctorotorParams["Izz"])

        # J matrix 
        self.J = np.array([[self.Ixx, 0, 0], [0, self.Iyy, 0], [0, 0, self.Izz]], dtype="float32")
        self.invJ = np.array([[1/self.Ixx, 0, 0], [0, 1/self.Iyy, 0], [0,0, 1/self.Izz]], dtype="float32")
        
        # Gazebo Publisher
        self.host = '127.0.0.1'
        self.port = 9002
        self.loop = asyncio.get_event_loop()
        self.running = False
        self.timeout = 30
        self.writer = self.loop.create_datagram_endpoint(
            lambda: ActionProtocol(),
            remote_addr=(self.host, self.port))
        _, self.ac_protocol = self.loop.run_until_complete(self.writer)
        
        # Send Gazebo message twice to receive accurate initial state values
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
        self.state, _ = self.loop.run_until_complete(self.ac_protocol.write(initialize))
        self.state, _ = self.loop.run_until_complete(self.ac_protocol.write(initialize))
        
        
    
    
    # Update function handles progressing the Gazebo Digital Twin and receiving the state.
    def update(self, msg_array):
        self.stepNum += 1
        
        rpm = np.zeros(8)
        for i in range(8):
            rpm[i] = round(msg_array[i] * 9.5493, 2)
        
        
        #print("Array of motor signals to send:" , rpm)
        
        msg = Vector8d()
        msg.motor_1 = rpm[0]
        msg.motor_2 = rpm[1]
        msg.motor_3 = rpm[2]
        msg.motor_4 = rpm[3]
        msg.motor_5 = rpm[4]
        msg.motor_6 = rpm[5]
        msg.motor_7 = rpm[6]
        msg.motor_8 = rpm[7]
        
        #Send UDP Pack to step world and return octocopter state
        self.state, _ = self.loop.run_until_complete(self.ac_protocol.write(msg))
        
        # Add constraint for z
        self.state[2] = max(self.state[2], 0)
    
    
    
    def get_velocity(self):
        return self.state[3:6]

    def get_position(self):
        return self.state[0:3]

    def get_angle(self):
        return self.state[6:9]
    
    def get_angle_vel(self):
        return self.state[9:12]

    def update_u(self, u):
        self.T = u[0]
        self.tau = u[1:4]

    def get_j_matrix(self):
        return self.J

    def get_state(self):
        return self.state

    def set_pos(self, x, y):
        self.state[0] = x
        self.state[1] = y

    def get_u(self):
        return self.u
