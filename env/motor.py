import scipy.integrate
import numpy as np

"""

Motor.py

This Script simulates a motor object class that can receive a voltage or current amount and
convert voltage into an angular speed based on the specifications of the 
Tarot T18 Octocopter motor model.


"""

class Motor:

	def __init__(self):
		self.req = 0.2799				# resisitence
		self.torque_constant = 0.0265	# torque constant Ke
		self.J = 5.0000e-05				# Inertia
		self.Df = 0						# Viscous Damping Coefficient
		self.static_friction = 0		# static friction
		self.motor_constant = 0.0265	# Motor constant Km

		self._original_params = dict(
			req=self.req,
			torque_constant=self.torque_constant,
			J=self.J,
			Df=self.Df,
			static_friction=self.static_friction,
			motor_constant=self.motor_constant)
		
		self.stepNum = 0
		self.omega = 0
		self.ode = scipy.integrate.ode(self.omega_dot_i).set_integrator('vode', method='bdf')
		
	
	def update(self, voltage, dt):
		
		# ----------------------------
		# 		UPDATE
		# Returns a motor's angular velocity moving one step in time 
		# with a given voltage. Takes in as parameters, voltage and sample rate
		# ----------------------------
		
		self.stepNum += 1
		self.v = voltage
		self.ode.set_initial_value(self.omega, 0)
		self.omega = self.ode.integrate(self.ode.t + dt)
		return self.omega
		

	def omega_dot_i(self, time, state):
		
		# ----------------------------
		#		OMEGA_DOT_I
		# Helper Method to calculate omega_dot for our ode integrator.
		# Can be written as a lambda function inside update for other shorter motors
		# ----------------------------
		
		rpm = self.omega * 9.5493
		
		t1 = self.motor_constant / self.req * self.v
		t2 = -((2.138e-08)*rpm**2 + (-1.279e-05)*rpm)
		t3 = -(self.motor_constant * self.torque_constant / self.req * self.omega)
		
		domega = (t1 + t2 + t3) / self.J
		
		return domega
	

	def fault(self, param: str, magnitude: float):
		if param in self._original_params:
			value = self._original_params[param]
			setattr(self, param, value * magnitude)
		else:
			raise AttributeError('Param %s cannot be faulted in motor.' % param)


	def unfault(self):
		for param, value in self._original_params.items():
			setattr(self, param, value)
