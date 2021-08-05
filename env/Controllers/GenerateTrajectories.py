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
	
	
	
	def Generate(self, currentX, currentY, targetX, targetY, velocity):
		
		start_location = [currentX , currentY]
		target_location = [targetX , targetY]
		x = np.array([currentX, targetX])
		y = np.array([currentY, targetY])
		
		desired_velocity = velocity	# meters per second
		
		# calculate total distance to cover
		total_distance = math.sqrt((targetX - currentX)**2 + (targetY - currentY)**2)
		
		# time interval between waypoints
		time_interval = total_distance / desired_velocity
		
		# total time to complete the mission
		total_time = time_interval + 0.25*time_interval
		
		
		
		# generate array of sample times: tSamples
		sampletimetraj = 1
		tSamples = []
		for i in np.arange(0, time_interval, sampletimetraj):
			tSamples.append(i)
		print(len(tSamples), " samples.")
		#print("Samples: ", tSamples)
		
		
		# Generate B-Spline path
		xx = np.linspace(currentX, targetX, len(tSamples), endpoint=False)
		yy = np.linspace(currentY, targetY, len(tSamples), endpoint=False)
		"""
		xmin, xmax = x.min(), x.max()
		if targetX > currentX:
			xx = np.linspace(xmin, xmax, len(tSamples), endpoint=False)
		else:
			xx = np.linspace(xmax, xmin, len(tSamples), endpoint=False)
		
		ymin, ymax = y.min(), y.max()
		if targetY > currentY:
			yy = np.linspace(ymin, ymax, len(tSamples), endpoint=False)
		else:
			yy = np.linspace(ymax, ymin, len(tSamples), endpoint=False)
		"""
		
		#t, c, k = interpolate.splrep(x, y, s=0, k=1)
		#spline = interpolate.BSpline(t, c, k, extrapolate=False)
		#plt.plot(xx, spline(xx), 'r', label='BSpline')
		
		#plt.plot(x, y, 'bo', label='Waypoints')
		#plt.grid()
		#plt.legend(loc='best')
		#plt.show()
		#[q,qd,qdd,pp] = bsplinepolytraj(path',timeinterval,tSamples)
		
		return xx, yy



"""
traj = GenerateTrajectories()	
xarray, yarray = traj.Generate( 0, 0 , 4, 4, velocity=1)
for i in range(len(xarray)):
	print(i, ". ( ", xarray[i], " , " , yarray[i] , " )") 

"""




