import asyncio
import time
import numpy as np
import codecs
from .proto.vector8d_pb2 import Vector8d

class GazeboPublisher: 

    def __init__(self, host='127.0.0.1', port=11345, timeout=30):
        self.host = host
        self.port = port
        self.loop = asyncio.get_event_loop()
        self.running = False
        self.timeout = timeout
        
        # Following 2 lines are to set up UDP version of Publisher
        self.writer = self.loop.create_datagram_endpoint(
            lambda: ActionProtocol(),
            remote_addr=(self.host, 9002))
        _, self.ac_protocol = self.loop.run_until_complete(self.writer)
    
    
    
    #---------------------------------
    #		Connect()
    # Connect a publisher to Gazebo to send motor values to a given topic, NOT over UDP.
    # Does the job of tarot_env_plugin, but is slower than UDP
    # Arguments:
    #	* topic: String representing what topic to publish to, e.g. "/gazebo/default/vector3d"
    #	* msg_format: String representing what kind of message that is being
    #	  published, needed for declaration of advertizer. e.g. "gazebo.msgs.Vector3d"
    #	* message: Message object to be published, of the corresponding type specified by msg_format
    #---------------------------------
    async def connect(self, topic, msg_format, message):
        connected = False
        for i in range(self.timeout):
            try:
                self.manager = await pygazebo.connect((self.host, self.port))
                connected = True
                break
            except Exception as e:
                pass
            await asyncio.sleep(1)

        if connected:
            print("\tPublisher connected to Gazebo.")
            self.topic = topic
            self.publisher = self.manager.advertise(self.topic, msg_format)
            self.running = True
            print("\tPublisher advertising to ", self.topic)
            await asyncio.sleep(1)
            self.manager._publishers[self.topic].publish(message)
            #while self.running:
                #self.manager._publishers[self.topic].publish(message)
                #await asyncio.sleep(0.1)
        else:
            raise Exception("Timeout connecting to Gazebo.")
    
    
    
    #-----------------------
    #	ConnectUDP:
    # UDP Version of Publisher, which sends messages to tarot_env plugin
    #-----------------------
    def connectUDP(self):
        _, self.ac_protocol = self.loop.run_until_complete(self.writer)
        
        
        
        


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




