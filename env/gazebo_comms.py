import asyncio
import time
from typing import Iterable

import google.protobuf.message
import pygazebo
import pygazebo.msg.v11 as msg
# This module is imported to patch asyncio so tasks can be added to the
# already running event loop if code is imported in jupyter notebook.
import nest_asyncio
nest_asyncio.apply()



class GazeboCommms: 

    def __init__(self, host='127.0.0.1', port=11345, timeout=30):
        self.host = host
        self.port = port
        self.loop = asyncio.get_event_loop()
        self.running = False
        self.timeout = timeout
        
        # Set up UDP transport for sending action packets
        future = self.loop.create_datagram_endpoint(
            lambda: UDPProtocol(),
            remote_addr=(self.host, 9002)
        )
        self.udp_protocol: UDPProtocol = \
            self.loop.run_until_complete(future)[1]

        # Set up a TCP/IP connection manager for interfacing w/ Gazebo
        self.manager: pygazebo.pygazebo.Manager = \
            asyncio.run(pygazebo.connect((host, port)))


    def send_UDP(self, message: google.protobuf.message.Message):
        return self.loop.run_until_complete(self.udp_protocol.write(message))


    def send_TCP(self, message: google.protobuf.message.Message, topic: str):
        try:
            return self.loop.run_until_complete(
                self.manager._publishers[topic].publish(message)
            )
        except KeyError:
            # If topic is not advertised for publication, start the advertisement
            # and then send the message
            self.loop.run_until_complete(
                self.manager.advertise(topic, message.DESCRIPTOR.full_name)
            )
            return self.loop.run_until_complete(
                self.manager._publishers[topic].publish(message)
            )
    


class UDPProtocol:
    # Sets up the protocol to be used by the UDP transport. Implements methods
    # called at various transport events.
    # See https://docs.python.org/3.9/library/asyncio-protocol.html#protocols
    
    def __init__(self):
        self.state_message = None
        self.packet_received = False
        self.exception = None
        self.send_time = None
        self.timeout = 30
        self.first = True
    
    
    def connection_made(self, transport: asyncio.DatagramTransport):
        self.transport = transport
    
    
    async def write(self, message: google.protobuf.message.Message):
        """ Write the motor values to the ESC and then return 
        the current sensor values and an exception if anything bad happend.
        
        Args:
            motor_vector: Vector8d object of motor values
        """
        self.packet_received = False
        
        # If statement to force UDP message to continue after first message.
        # For some reason, python publisher does not receive the first message and gets stuck
        # So, don't worry about the first message, since it will be updated the next 0.001 second.
        # if self.first == True:
        #     self.packet_received = True
        #     self.state_message = "-1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1"
        #     self.first = False
        
        self.send_time = time.time()
        # Send the action message
        self.transport.sendto(message.SerializeToString())

        # And then wait for response
        while not self.packet_received:
            if self.exception:
                return (None, self.exception)
            elif time.time() > self.send_time + self.timeout:
                return (None, Exception("Timeout reached waiting for response."))
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
        self.state_message = StatePacket.decode(data)


    def connection_lost(self, exc):
        print("Socket closed, stop the event loop")
        loop = asyncio.get_event_loop()
        loop.stop()
    


class StatePacket:
    
    @classmethod
    def decode(cls, data):
       state_string = str(data, 'utf-8')
       
       state_string_array = state_string.split()
       state = [float(numeric_string) for numeric_string in state_string_array]
       
       return state  




