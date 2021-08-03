import gym
from gym import error, spaces, utils
from gym.utils import seeding
from gym.envs.classic_control  import rendering
from .Octocopter import Octocopter
from .Actuation import ControlAllocation
import pandas as pd
import numpy as np
import math
from .motor import Motor
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
import time

def hx(x):
    return x

class OctorotorBaseEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    # Initialize the Octorotor
    # Simulation Parameters consist of the following and are passed via dictionary
    # g - gravity
    # mass - mass of octorotor
    # d - friction coefficient
    # b - rotor thrust constant
    # l - length of rotor arms (all equal)
    # omega_max - max angular velocity
    # Ixx - x moment of intertia
    # Iyy - y moment of inertia
    # Izz - z moment of inertia
    # dt  - time_Step of simulation
    # Motor - a motor object that inherits the Motor base class: Must have constructor and update functions implemented
    # motorController - A controller object that inherits the controller base class: Must have constructor and output methods implemented
    def __init__(self, OctorotorParams):
        super(OctorotorBaseEnv, self).__init__()
        # Octorotor Params
        self.octorotor = Octocopter(OctorotorParams)
        self.state = self.octorotor.get_state()
        self.xref = 0
        self.yref = 0
        #self.xrefarr = pd.read_csv("~/gymfc/scripts/Controllers/Paths/EpathX5.csv", header=None).iloc[:, 1]
        #self.yrefarr = pd.read_csv("~/gymfc/scripts/Controllers/Paths/EpathY5.csv", header=None).iloc[:, 1]
        self.xrefarr = 0
        self.yrefarr = 0
        
        self.allocation = ControlAllocation(OctorotorParams)
        self.resistance = np.full(8, OctorotorParams["resistance"])
        self.dt = OctorotorParams["dt"]
        
        # Declare 8 Individual motors
        self.motor_1 = OctorotorParams["motor"]
        self.motor_2 = OctorotorParams["motor"]
        self.motor_3 = OctorotorParams["motor"]
        self.motor_4 = OctorotorParams["motor"]
        self.motor_5 = OctorotorParams["motor"]
        self.motor_6 = OctorotorParams["motor"]
        self.motor_7 = OctorotorParams["motor"]
        self.motor_8 = OctorotorParams["motor"]
        
        self.motorController = OctorotorParams["motorController"]
        self.posc = OctorotorParams["positionController"]
        self.attc = OctorotorParams["attitudeController"]
        self.altc = OctorotorParams["altitudeController"]
        self.OctorotorParams = OctorotorParams
        self.omega = np.zeros(8)
        self.step_count = 0
        self.total_step_count = OctorotorParams["total_step_count"]
        self.zref = 2
        self.psiref = np.zeros(3)
        self.reward_discount = OctorotorParams["reward_discount"]
        # OpenAI Gym Params
        # State vector
        # state[0:2] pos
        # state[3:5] vel
        # state[6:8] angle
        # state[9:11] angle vel
     
        #self.observation_space = spaces.Box(np.full(1, -np.inf, dtype="float32"), np.full(1, np.inf, dtype="float32"), dtype="float32")

        #self.action_space = spaces.Box(np.array([0.1, 0.1, 0.1, 0.1]), np.array([1, 1, 1, 1]), dtype="float32")
        self.viewer = None
        # introduce filtering for parameter estimator
        #points = MerweScaledSigmaPoints(6, alpha=.1, beta=2, kappa=0)
        #self.kf = UnscentedKalmanFilter(dim_x=6, dim_z=6, dt=OctorotorParams["dt"], fx=self.fx, hx=hx, points=points)
        #self.kf.x = np.zeros(6)
    
    
    
    def step(self, xarr , yarr, z_ref, steps):
        # Run through control allocation, motor controller, motor, and octorotor dynamics in this order
        reward = 0
        #xarr = [0, 1, 4, 4]
        #yarr = [0, 0, 2, 4]
        
        #self.posc.update_params(np.array(action)) # why do this if we initialized the posc gains?
        k = 0
        refguessxarr = []
        refguessyarr = []
        xestimate = []
        yestimate = []
        psi0 = []
        psi1 = []
        self.zref = z_ref
        self.index = 0
        
        while k < 150000:
            if k % steps == 0:
                if self.index == 4:
            	    self.index = 1
                self.xref = xarr[self.index]
                self.yref = yarr[self.index]
                self.index+=1
                print("Current Location: (" , self.state[0], " , " , self.state[1] , " , ", self.state[2] , ")")
                print("Target Coordinants: (" , self.xref, " , " , self.yref, ").")
                
            
            targetValues = {"xref": self.xref, "yref": self.yref} #target coordinants for posc
            #print("Current Location: (" , self.state[0], " , " , self.state[1] , " , ", self.state[2] , ")")
            
            #print("Location: (" , self.state[0], " , " , self.state[1] , " , ", self.state[2] , ")")         
            # Update Position Controller
            
            self.psiref[1], self.psiref[0] = self.posc.output(self.state, targetValues)
            
            # Update Attitude and Altitude Controller
            tau_des = self.attc.output(self.state, self.psiref)
            T_des = self.altc.output(self.state, self.zref)
            
            # Calculate omega reference using calculated thrust and euler angles
            udes = np.array([T_des, tau_des[0], tau_des[1], tau_des[2]], dtype="float32")
            omega_ref = self.allocation.get_ref_velocity(udes)
            #print(omega_ref)
            
            # Send omega reference to motorController to calculate voltage array
            voltage = self.motorController.output(self.omega, omega_ref)
            #print(voltage)
            
            # Send Voltage array to motors to produce RPM array
            if self.dt == 0.01:
                self.omega[0] = self.motor_1.update(voltage[0], 0.01)
                self.omega[1] = self.motor_2.update(voltage[1], 0.01)
                self.omega[2] = self.motor_3.update(voltage[2], 0.01)
                self.omega[3] = self.motor_4.update(voltage[3], 0.01)
                self.omega[4] = self.motor_5.update(voltage[4], 0.01)
                self.omega[5] = self.motor_6.update(voltage[5], 0.01)
                self.omega[6] = self.motor_7.update(voltage[6], 0.01)
                self.omega[7] = self.motor_8.update(voltage[7], 0.01)
                i = 0
                while i < 10:
                    u = self.allocation.get_u(self.omega)
                    self.octorotor.update_u(u)
                    self.octorotor.update(self.omega)
                    self.state = self.octorotor.get_state()
                    k += 1
                    i += 1
            elif self.dt == 0.001:
                self.omega[0] = self.motor_1.update(voltage[0], 0.001)
                self.omega[1] = self.motor_2.update(voltage[1], 0.001)
                self.omega[2] = self.motor_3.update(voltage[2], 0.001)
                self.omega[3] = self.motor_4.update(voltage[3], 0.001)
                self.omega[4] = self.motor_5.update(voltage[4], 0.001)
                self.omega[5] = self.motor_6.update(voltage[5], 0.001)
                self.omega[6] = self.motor_7.update(voltage[6], 0.001)
                self.omega[7] = self.motor_8.update(voltage[7], 0.001)
                u = self.allocation.get_u(self.omega)
                self.octorotor.update_u(u)
                self.octorotor.update(self.omega)
                self.state = self.octorotor.get_state()
                k += 1
                 
            
            
            
            #xarr.append(self.state[0])
            #yarr.append(self.state[1])
            #psi0.append(self.psiref[0])
            #psi1.append(self.psiref[1])
            
            
        return [self.res], reward, True, {"xerror": xarr, "yerror": yarr, "xref": refguessxarr, "yref": refguessyarr, "xestimate": xestimate, "yestimate": yestimate, "psi0": psi0, "psi1": psi1}

    def reset(self):
        OctorotorParams = self.OctorotorParams
        self.octorotor = Octocopter(OctorotorParams) 
        #self.octorotor.set_pos((b- a) * np.random.random_sample() + a, (b-a)*np.random.random_sample()+a)
        self.allocation = ControlAllocation(OctorotorParams)
        self.omega = np.zeros(8)
        self.dt = OctorotorParams["dt"]
        self.motor = OctorotorParams["motor"]
        self.motor.reset()
        self.motorController = OctorotorParams["motorController"]
        # between 0.7 and 1.7
        # two motor between 0.5 and 0.9
        #self.res = 0.5
        #self.motor.update_r(self.res, 2)
        #self.motor.update_r2(self.res, 5)
        self.step_count = 0
        self.total_step_count = OctorotorParams["total_step_count"]
        self.viewer = None
        self.xref = 5
        self.yref = 0
        self.index = 0
        self.psiref = np.zeros(3)
        self.state = self.octorotor.get_state()
        self.errors = [self.xref-self.state[0], self.yref-self.state[1], self.zref-self.state[2]]
        self.eulererrors = [self.state[3] - self.psiref[0], self.state[4]-self.psiref[1], self.state[5]-self.psiref[2]]
        state = np.append(self.errors, self.eulererrors)
        #guess = np.random.normal(self.res, 0.1)
        self.res = 0.2799
        return self.res

    def render(self,mode='human'):
        xref = self.xref
        yref = self.yref
        screen_width = 600
        screen_height = 600
        # Set width to 100x100
        world_width = 600
        scale = screen_width/world_width
        rotorradius = 4
        armwidth = 1
        armlength = self.OctorotorParams["l"]*scale + rotorradius
        if self.viewer is None:
            # build Octorotor
            self.viewer = rendering.Viewer(screen_width, screen_height)
            rotor = rendering.make_circle(radius=rotorradius)
            self.rotortrans = rendering.Transform()
            rotor.add_attr(self.rotortrans)
            rotor.set_color(1, 0, 0)
            self.viewer.add_geom(rotor)
            self.add_arm((0, 0), (armlength, 0))
            self.add_arm((0, 0), (-armlength, 0))
            self.add_arm((0, 0), (0, armlength))
            self.add_arm((0, 0), (0, -armlength))
            self.add_arm((0, 0), (armlength, armlength))
            self.add_arm((0, 0), (-armlength, armlength))
            self.add_arm((0, 0), (-armlength, -armlength))
            self.add_arm((0, 0), (armlength, -armlength))
            # Build ref Point
            refPoint = rendering.make_circle(radius = rotorradius)
            self.refPointTrans = rendering.Transform()
            refPoint.add_attr(self.refPointTrans)
            refPoint.set_color(0, 0, 1)
            self.refPointTrans.set_translation(xref*scale+screen_width/2, yref*scale+screen_width/2)
            self.viewer.add_geom(refPoint)
            
        if self.state is None:
            return None
        # Translate Rotor according to x, y
        x = self.state[0]
        y = self.state[1]
        rotorx = x*scale + screen_width/2.0
        rotory = y*scale + screen_width/2.0
        self.rotortrans.set_translation(rotorx, rotory)
        self.refPointTrans.set_translation(xref*scale+screen_width/2, yref*scale+screen_width/2)
        return self.viewer.render(return_rgb_array=mode == 'rgb_array')

    def add_arm(self, start, end):
        arm = rendering.Line(start=start, end=end)
        arm.add_attr(self.rotortrans)
        self.viewer.add_geom(arm)

    def close(self):
        if self.viewer:
            self.viewer.close()
            self.viewer = None

    def reward(self):
        error = math.sqrt((self.xref-self.state[0])*(self.xref-self.state[0]) + (self.yref-self.state[1]) * (self.yref-self.state[1])+ (self.zref-self.state[2]) * (self.zref-self.state[2]))
        return (-error+10)/10

    def episode_over(self):
        error = math.sqrt((self.xref-self.state[0])*(self.xref-self.state[0]) + (self.yref-self.state[1]) * (self.yref-self.state[1]) + (self.zref-self.state[2]) * (self.zref-self.state[2]))
        return self.step_count >= 2000 or error > 10

    def get_state(self):
        return self.state

    def get_xerror(self):
        return self.state[0]

    def get_yerror(self):
        return self.state[1]
