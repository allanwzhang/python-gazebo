import numpy as np
from Controllers.proto.vector8d_pb2 import Vector8d
from Controllers.GazeboPublisher import GazeboPublisher
from motor import Motor
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
    stepsize = 1
    maxsteps = 1000
    maxtilt = np.asarray([np.pi / 4, np.pi / 4, np.pi / 4])
    maxdisp = np.asarray([0.5, 0.5, 0.5])
    maxvoltage = 20.
    toltilt = np.asarray([np.pi / 32, np.pi / 32, np.pi / 32])
    tolangvel = np.asarray([5 * np.pi / 180, 5 * np.pi / 180, 5 * np.pi / 180])
    # minstepreward = -(np.sum(maxtilt**2) + np.sum(maxdisp**2))
    absminstepreward = np.linalg.norm(maxtilt)

    HOVER_ACTION = np.ones(8) * 1.2


    def __init__(self, gym_fc_obs=True) -> None:
        super().__init__()
        self.initial_obs = None
        self.action_space = gym.spaces.Box(
            low=-np.ones(8, dtype=np.float32),
            high=np.ones(8, dtype=np.float32)
        )
        self.gym_fc_obs = gym_fc_obs
        if not gym_fc_obs:
            # pos, vel, att, ang vel
            self.observation_space = gym.spaces.Box(
                low=-np.inf,
                high=np.inf,
                dtype=np.float32,
                shape=(12,)
            )
        else:
            # 8 motor velocities + 3 axis errors
            self.observation_space = gym.spaces.Box(
                low=-np.inf,
                high=np.inf,
                dtype=np.float32,
                shape=(11,)
            )
        self.publisher = GazeboPublisher(self.MASTER_TCP_IP, self.MASTER_TCP_PORT)
        self.motors = [Motor() for _ in range(8)]
        self.t = 0
        self.targetatt = None
        self.targetpos = None
        self.targetlinvel = None
        self.targetangvel = None
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
        return np.asarray(state, dtype=np.float32)


    def step(self, action: np.ndarray):
        # Map [-1, 1] to [0, self.maxvoltage]
        action = np.clip((action + 1) * self.maxvoltage / 2, a_min=0, a_max=self.maxvoltage)
        state = self._send(action, steps=self.stepsize)

        tilt = np.abs(state[6:9])
        violation = np.any(tilt > self.maxtilt)
        goal_att = np.all(tilt <= self.toltilt)
        done = (self.t >= self.maxsteps) or goal_att or violation

        if self.gym_fc_obs:
            motor_vel = [m.omega for m in self.motors]
            att_err = state[6:9]
            motor_vel.extend(att_err)
            obs = np.asarray(motor_vel, dtype=np.float32)  # [motor speeds, attitude error]
        else:
            obs = state
        self.t += 1

        reward = self.reward(obs, state)
        if violation:
            reward -= self.absminstepreward * (self.maxsteps - self.t) / 10
        elif goal_att:
            # positive reward for getting to goal att early
            reward += self.absminstepreward * (self.maxsteps - self.t) / 10
            # negative reward for getting there at high velocity
            # reward -= np.linalg.norm(state[9:12], 2) * (self.maxsteps - self.t)

        return obs, reward, done, {}


    def reset(self):
        for motor in self.motors:
            motor.reset()
        msg = Vector8d()
        msg.motor_1 = -1
        msg.motor_2 = -1
        msg.motor_3 = -1
        msg.motor_4 = -1
        msg.motor_5 = -1
        msg.motor_6 = -1
        msg.motor_7 = -1
        msg.motor_8 = -1

        state, _ = self.publisher.loop.run_until_complete(self.publisher.ac_protocol.write(msg))
        self.pos0 = state[:3]
        self.att0 = state[6:9]
        if self.gym_fc_obs:
            motor_vel = [m.omega for m in self.motors]
            att_err = state[6:9]
            motor_vel.extend(att_err)
            self.initial_obs = np.asarray(motor_vel, dtype=np.float32)  # [motor speeds, attitude error]
        else:
            self.initial_obs = np.asarray(state, dtype=np.float32)
        self.t = 0
        return self.initial_obs


    def fault(self, motor, param, magnitude):
        self.motors[motor].fault(param, magnitude)


    def unfault(self):
        for motor in self.motors:
            motor.unfault()


    def reward(self, obs: np.ndarray, state: np.ndarray) -> float:
        pos, vel, att, ang_vel = state[:3], state[3:6], state[6:9], state[9:]
        # Reward is to maintain original position, but have 0 velocity and attitude
        # r = -(pos - self.pos0)**2 - (att - 0)**2 - (vel - 0)**2 - (ang_vel - 0)**2
        # r = -(pos - self.pos0)**2 - (att - 0)**2
        # r = -(att - 0)**2
        r = -np.linalg.norm(att) / 3
        return r



dict_space = gym.spaces.Dict({
    'position': gym.spaces.Box(low=-np.inf, high=-np.inf, dtype=np.float32, shape=(3,)),
    'velocity': gym.spaces.Box(low=-np.inf, high=-np.inf, dtype=np.float32, shape=(3,)),
    'orientation': gym.spaces.Box(low=-np.inf, high=-np.inf, dtype=np.float32, shape=(3,)),
    'angular_velocity': gym.spaces.Box(low=-np.inf, high=-np.inf, dtype=np.float32, shape=(3,))
})

