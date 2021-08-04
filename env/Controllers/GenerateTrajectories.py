import math
import numpy as np
import scipy.interpolate as interpolate
import matplotlib.pyplot as plt



# 		Generate Trajectories.
# Function to generate an array of x and y coordinates
# to serve as intermediate waypoints between the octocopter's current location
# and a target location.

class GenerateTrajectories():
	
	def __init__(self):
		pass
	
	
	
	def Generate(self, currentX, currentY, targetX, targetY):
		
		start_location = [currentX , currentY]
		target_location = [targetX , targetY]
		x = np.array([currentX, targetX])
		y = np.array([currentY, targetY])
		plt.plot(x, y, 'bo', label='Waypoints')
		
		desired_velocity = 1	# meters per second
		
		# calculate total distance to cover
		total_distance = math.sqrt((targetX - currentX)**2 + (targetY - currentY)**2)
		
		# time interval between waypoints
		time_interval = total_distance / desired_velocity
		
		# total time to complete the mission
		total_time = time_interval + 0.25*time_interval
		
		
		
		# generate array of sample times: tSamples
		sampletimetraj = 1
		tSamples = []
		for i in np.arange(sampletimetraj, time_interval, sampletimetraj):
			tSamples.append(i)
		print("Samples: ", tSamples)
		
		
		# Generate B-Spline path
		xmin, xmax = x.min(), x.max()
		xx = np.linspace(xmin, xmax, len(tSamples))
		print(xx)
		t, c, k = interpolate.splrep(x, y, s=0, k=1)
		spline = interpolate.BSpline(t, c, k, extrapolate=False)
		plt.plot(xx, spline(xx), 'r', label='BSpline')
		
		x_array = []
		y_array = []
		
		# Populate Xarray and Yarray with coordinates along the path at each sample time.
		
		plt.grid()
		plt.legend(loc='best')
		plt.show()
		#[q,qd,qdd,pp] = bsplinepolytraj(path',timeinterval,tSamples)



	
traj = GenerateTrajectories()	
traj.Generate( 5, 5, 20, 20)






