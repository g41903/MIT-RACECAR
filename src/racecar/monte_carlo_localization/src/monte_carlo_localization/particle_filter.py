import numpy as np
import rospy

pose_dtype = np.dtype(dict(
	names=['x', 'y', 'theta', 'xy'],
	formats=[np.float64, np.float64, np.float64, (np.float64, 2)],
	offsets=[0, 8, 16, 0]
))

def resample(particles, weights, M):
    weights = np.asarray(weights)
    weights /= np.sum(weights)
    # Generates a random integer "which" from a given array which starts from 0 to the number of particles, 
    # and the probabiltiy distribution of "which" is according to its distribution of weight to the sum of weight 
    which = np.random.choice(len(particles), size=M, p=weights)
    return particles[which]
