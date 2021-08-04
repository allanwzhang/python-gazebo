from numpy.linalg import inv
import numpy as np
import math
from numba import jit

@jit
def _get_ref_velocity(udes, invAF, E, omegamax, version):
    
    omega_ref = np.zeros(8, dtype="float32")
    
    if (version==0):
        omega_s_f = invAF.dot(udes)
        omega_s = E.dot(omega_s_f)
        for idx, item in enumerate(omega_s):
            # why are we taking the square root of our reference velocity??
            #omega_ref[idx] = math.sqrt(max(0, min(item, math.pow(omegamax, 2.0))))
            omega_ref[idx] = max(0, min(item, omegamax))
        
    else:
        omega_s = invAF.dot(udes)
        for idx, item in enumerate(omega_s):
            #omega_ref[idx] = math.sqrt(max(0, min(item, math.pow(omegamax, 2.0))))
            omega_ref[idx] = max(0, min(item, omegamax))
        
    return omega_ref

@jit
def _get_u(omega, A):
    return A.dot(np.square(omega.astype(np.float32)))

class ControlAllocation:
    def __init__(self, ControlAllocationParams, version=0):
        self.version = version
        b = ControlAllocationParams["b"]
        d = ControlAllocationParams["d"]
        l = ControlAllocationParams["l"]
        self.omegamax = np.float32(ControlAllocationParams["omegamax"])
        sqrt2 = math.sqrt(2)
        angsm = math.cos(math.pi/8)
        anglg = math.cos(3*math.pi/8)
        
        if self.version == 0:
            self.A = np.array([[b, b, b, b, b, b, b, b], [b*l, sqrt2/2*b*l, 0, -sqrt2/2*b*l, -b*l, -sqrt2/2*b*l, 0, sqrt2/2*b*l], [0, -sqrt2/2*b*l, -b*l, -sqrt2/2*b*l, 0, sqrt2/2*b*l, b*l, sqrt2/2*b*l], [-d, d, -d, d, -d, d, -d,d]], dtype="float32")
            
            self.AF = np.array([[2*b, 2*b, 2*b, 2*b], [b*l, 0, -b*l, 0], [-b*l, -sqrt2*b*l, b*l, sqrt2*b*l], [-2*d, 2*d, -2*d, 2*d]], dtype="float32")
            
            self.invAF = inv(self.AF)
            
            self.E = np.array([[1, 0, 1, 0, 0, 0, 0, 0], [0, 1, 0, 1, 0, 0, 0, 0], [0, 0, 0, 0, 1, 0, 1, 0], [0, 0, 0, 0, 0, 1, 0, 1]], dtype="float32").transpose()
        
        else:
            self.A = np.array([[b, b, b, b, b, b, b, b], 
            [b*l*anglg, b*l*angsm, -b*l*angsm, -b*l*anglg, -b*l*anglg, -b*l*angsm, b*l*angsm, b*l*anglg], 
            [-b*l*angsm, -b*l*anglg, -b*l*anglg, -b*l*angsm, b*l*angsm, b*l*anglg, b*l*anglg, b*l*angsm], 
            [d, -d, d, -d, d, -d, d, -d]], dtype="float32")
            
            self.E = np.array([[1, 0, 1, 0, 0, 0, 0, 0], [0, 1, 0, 1, 0, 0, 0, 0], [0, 0, 0, 0, 1, 0, 1, 0], [0, 0, 0, 0, 0, 1, 0, 1]], dtype="float32").transpose()
            
            self.invAF = np.linalg.pinv(self.A) 
        
        
        
        
        
        

    def get_ref_velocity(self, udes):
        return _get_ref_velocity(udes, self.invAF, self.E, self.omegamax, self.version)


    def get_u(self, omega):
        return _get_u(omega, self.A)
