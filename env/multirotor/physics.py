from typing import Iterable, Tuple

import numpy as np
from numba import njit
from scipy.optimize import fsolve



@njit
def thrustEqn(vi, *prop_params):
    R,A,rho,a,b,c,eta,theta0,theta1,u,v,w,Omega = prop_params
    
    # Calculate local airflow velocity at propeller with vi, v'
    vprime = np.sqrt(u**2 + v**2 + (w - vi)**2)
    
    # Calculate Thrust averaged over one revolution of propeller using vi
    Thrust = 1/4 * rho * a * b * c * R * \
        ( (w - vi) * Omega * R + 2/3 * (Omega * R)**2 * (theta0 + 3/4 * theta1) + \
          (u**2 + v**2) * (theta0 + 1/2 * theta1) )
    
    # Calculate residual for equation: Thrust = mass flow rate * delta Velocity
    residual = eta * 2 * vi * rho * A * vprime - Thrust
    return residual



def thrust(
    prop_speed, airstream_velocity, R, A, rho, a, b, c, eta, theta0, theta1
) -> float:
    u, v, w = airstream_velocity
    # Convert commanded RPM to rad/s
    # omega = 2 * np.pi / 60 * prop_speed # Commented out - already in rad/s
    omega = prop_speed
    vi_guess = airstream_velocity[2]
    
    #Collect propeller config, state, and input parameters
    prop_params = (R,A,rho,a,b,c,eta,theta0,theta1,u,v,w,omega)
    
    # Numerically solve for propeller induced velocity, vi
    # using nonlinear root finder, fsolve, and prop_params
    # TODO: numba jit gives error for fsolve ('Untyped global name fsolve')
    vi = fsolve(thrustEqn, vi_guess, args=prop_params, xtol=1e-4)[0]
    
    # Plug vi back into Thrust equation to solve for T
    vprime = np.sqrt(u**2 + v**2 + (w - vi)**2)
    thrust = eta * 2 * vi * rho * A * vprime
    return thrust



@njit
def torque(
    position_vector: np.ndarray, force: np.ndarray,
    moment_of_inertia: float, prop_angular_acceleration: float,
    drag_coefficient: float, prop_angular_velocity: float,
    clockwise: int
) -> np.ndarray:
    # TODO: See here
    # https://andrew.gibiansky.com/downloads/pdf/Quadcopter%20Dynamics,%20Simulation,%20and%20Control.pdf
    # Total moments in the body frame
    # yaw moments
    # tau = I . d omega/dt
    tau_rot = (
        # clockwise * moment_of_inertia * prop_angular_acceleration + 
        clockwise * drag_coefficient * prop_angular_velocity**2
    )
    # tau = r x F
    tau = np.cross(position_vector, force)
    # print(moment_of_inertia, prop_angular_acceleration)
    tau[2] = tau[2] + tau_rot
    return tau



@njit
def apply_forces_torques(
    forces: np.ndarray, torques: np.ndarray, x: np.ndarray, g: float, mass: float,
    inertia_matrix: np.matrix, inertia_matrix_inverse: np.matrix
) -> np.ndarray:
    # Store state variables in a readable format
    xI = x[0]       # Inertial frame positions
    yI = x[1]
    zI = x[2]
    ub = x[3]       # linear velocity along body-frame-x-axis
    vb = x[4]       # linear velocity along body-frame-y-axis
    wb = x[5]       # linear velocity along body-frame-z-axis (down is positive)
    phi = x[6]      # Roll
    theta = x[7]    # Pitch
    psi = x[8]      # Yaw
    p = x[9]        # body-frame-x-axis rotation rate
    q = x[10]        # body-frame-y-axis rotation rate
    r = x[11]        # body-frame-z-axis rotation rate
    
    # Pre-calculate trig values
    cphi = np.cos(phi);   sphi = np.sin(phi)    # roll
    cthe = np.cos(theta); sthe = np.sin(theta)  # pitch
    cpsi = np.cos(psi);   spsi = np.sin(psi)    # yaw

    fx, fy, fz = forces
    tx, ty, tz = torques
    I = inertia_matrix
    I_inv = inertia_matrix_inverse
    
    # Calculate the derivative of the state matrix using EOM
    xdot = np.zeros_like(x)
    # Position
    xdot[0] = cthe*cpsi*ub + (-cphi * spsi + sphi*sthe*cpsi) * vb + \
        (sphi*spsi+cphi*sthe*cpsi) * wb  # = xIdot
        
    xdot[1] = cthe*spsi * ub + (cphi*cpsi+sphi*sthe*spsi) * vb + \
        (-sphi*cpsi+cphi*sthe*spsi) * wb # = yIdot
        
    xdot[2] = (-sthe * ub + sphi*cthe * vb + cphi*cthe * wb) # = zIdot
    #  Velocity
    xdot[3] = 1/mass * (fx) + g * sthe + r * vb - q * wb  # = udot
    xdot[4] = 1/mass * (fy) - g * sphi * cthe - r * ub + p * wb # = vdot
    xdot[5] = 1/mass * (fz) - g * cphi * cthe + q * ub - p * vb # = wdot

    # Orientation
    xdot[6] = p + (q*sphi + r*cphi) * sthe / cthe  # = phidot
    xdot[7] = q * cphi - r * sphi  # = thetadot
    xdot[8] = (q * sphi + r * cphi) / cthe  # = psidot

    # Angular rate
    gyro = np.cross(x[9:12], I @ x[9:12])
    xdot[9:12] = I_inv @ (torques - gyro) # TODO: verify whether it is + or - gyro
    
    return xdot
