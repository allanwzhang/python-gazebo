import time
from datetime import datetime
import os
from pathlib import Path
import numpy as np
from Controllers.proto.vector8d_pb2 import Vector8d
from Controllers.GazeboPublisher import GazeboPublisher
from motor import Motor
import pandas as pd
import gym

# # Following variables are for publishing on the Gazebo Vector3d protobuff topic
# MASTER_TCP_IP   = '127.0.0.1'
# MASTER_TCP_PORT = 11345
# # Create UDP Publisher to communicate with Tarot_Env_Plugin
# publisher = GazeboPublisher.GazeboPublisher(MASTER_TCP_IP, MASTER_TCP_PORT)



class Octorotor(gym.Env):

    MASTER_TCP_IP   = '127.0.0.1'
    MASTER_TCP_PORT = 11345
    dt = 1e-3
    stepsize = 100

    HOVER_ACTION = np.ones(8) * 11.68


    def __init__(self) -> None:
        super().__init__()
        self.action_space = gym.spaces.Box(
            low=np.zeros(8, dtype=np.float32),
            high=np.ones(8, dtype=np.float32) * 20
        )
        self.observation_space = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            dtype=np.float32,
            shape=(12,)
        )
        self.publisher = GazeboPublisher(self.MASTER_TCP_IP, self.MASTER_TCP_PORT)
        self.motors = [Motor() for _ in range(8)]
        self._setup()


    def _setup(self):
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
        state, _ = self.publisher.loop.run_until_complete(self.publisher.ac_protocol.write(initialize))
        state, _ = self.publisher.loop.run_until_complete(self.publisher.ac_protocol.write(initialize))


    def _send(self, voltages: np.ndarray, steps=1):
        for s in range(steps):
            angular_vels = [m.update(v, self.dt) for m, v in zip(self.motors, voltages)]
            rpms = [round(omega[0] * 9.5493) for omega in angular_vels]
            msg = Vector8d()
            msg.motor_1 = rpms[0]
            msg.motor_2 = rpms[1]
            msg.motor_3 = rpms[2]
            msg.motor_4 = rpms[3]
            msg.motor_5 = rpms[4]
            msg.motor_6 = rpms[5]
            msg.motor_7 = rpms[6]
            msg.motor_8 = rpms[7]

            state, _ = self.publisher.loop.run_until_complete(self.publisher.ac_protocol.write(msg))
        return state


    def step(self, action: np.ndarray):
        return self._send(action, steps=self.stepsize)


    def reset(self):
        state = self._send(np.ones(8) * -1)
        return state


    def fault(self, motor, param, magnitude):
        self.motors[motor].fault(param, magnitude)


    def unfault(self):
        for motor in self.motors:
            motor.unfault()


    def reward(self, state: np.ndarray):
        pass



dict_space = gym.spaces.Dict({
    'position': gym.spaces.Box(low=-np.inf, high=-np.inf, dtype=np.float32, shape=(3,)),
    'velocity': gym.spaces.Box(low=-np.inf, high=-np.inf, dtype=np.float32, shape=(3,)),
    'orientation': gym.spaces.Box(low=-np.inf, high=-np.inf, dtype=np.float32, shape=(3,)),
    'angular_velocity': gym.spaces.Box(low=-np.inf, high=-np.inf, dtype=np.float32, shape=(3,))
})

