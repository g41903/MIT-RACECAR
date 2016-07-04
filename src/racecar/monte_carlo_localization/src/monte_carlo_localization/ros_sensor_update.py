import numpy as np
from geometry_msgs.msg import Pose2D
from ctypes import *
from numpy.ctypeslib import ndpointer

lib = cdll.LoadLibrary('libmonte_carlo_localization.so')
fun = lib.calc_line
fun.restype = None
fun.argtypes = [ndpointer(c_int8, flags="C_CONTIGUOUS"),c_size_t,c_size_t,c_size_t,c_size_t,POINTER(c_size_t),POINTER(c_size_t)]

def calc_line_c(start, target, map):
	tx = c_size_t(target[0])
	ty = c_size_t(target[1])
	(xlen,ylen) = map.grid.shape

	fun(map.grid, xlen, ylen, start[0], start[1], tx, ty)

	return ty.value,tx.value

def sensor_update(map, reading, particles, scan_to_link_tf, step=1):
	"""Given a map object, a LaserScan reading, and a particle returns weight for particle"""

	# pull out arrays of values
	N = len(reading.ranges)
	angles = reading.angle_min + reading.angle_increment * np.arange(0,N,step)
	ranges = np.asarray(reading.ranges)[0:N:step]

	# filter invalid
	valid = (reading.range_min <= ranges) & (ranges <= reading.range_max)
	angles = angles[valid]
	ranges = ranges[valid]

	# indexed by [part_i, scan_i]
	expected_ranges = calc_ranges(map, particles[...,np.newaxis], reading.range_max, angles, scan_to_link_tf)


	errors = (ranges - expected_ranges)**2

	# points far away are less dense and therefore under-represented. Correct for this
	errors *= expected_ranges

	# average over scan axis
	err = np.sum(errors, axis=1)

	# don't exponentiate yet
	sigma = 200
	log_prob = 0.5 * err/sigma**2

	# prevent overflow
	log_prob -= np.min(log_prob)

	return np.exp(-log_prob)

def stack(arrs):
	"""Y U NO numpy 1.10..."""
	return np.concatenate([a[...,np.newaxis] for a in arrs], axis=-1)

def calc_ranges(map, particle, max_range, angles, scan_to_link_tf):
	"""
	map:       a mcl.Map object
	particle:  an ndarray<pose> of one or more particles
	max_range: the maximum range to cast the rays
	angles:    an ndarray<float> of one or more ranges

	If particle and angles are both ndarrays, then the result will be the
	be of the shape expected by broadcasting. Both can be of arbitrary shape
	"""
	# turn points to 3D
	points = np.empty(particle.shape + (4,))
	points[...,0] = particle.x
	points[...,1] = particle.y
	points[...,2] = 0
	points[...,3] = 1

	# calculate end-points in 3D, for all angles
	end_x = particle.x + max_range * np.cos(particle.theta + angles)
	end_y = particle.y + max_range * np.sin(particle.theta + angles)
	ends = np.empty(end_x.shape + (4,))
	ends[...,0] = end_x
	ends[...,1] = end_y
	ends[...,2] = 0
	ends[...,3] = 1

	points = points.dot(scan_to_link_tf.T)
	ends = ends.dot(scan_to_link_tf.T)

	# these return tuples, which are useful for indexing...
	starts = map.index_at(points)
	max_targets = map.index_at(ends) #(is, js)

	# ...but not for iterating, so join em back into arrays!
	starts = stack(starts)
	max_targets = stack(max_targets)

	# here be dragon!
	starts, max_targets = np.broadcast_arrays(starts, max_targets)

	# cast all the rays
	for indices in np.ndindex(max_targets.shape[:-1]):
		max_targets[indices] = calc_line_c(starts[indices], max_targets[indices], map)

	# convert pixel indices to 3D coords
	targets = map.pos_at(max_targets[...,0], max_targets[...,1])

	# subtract from the start positions
	return np.hypot(targets[...,0] - particle.x, targets[...,1] - particle.y)

def calc_line(start, target, map):
	""" Takes a pair of (i, j) tuples of map indices and a map """
	""" Returns the real world point at the farthest range """
	dx = abs(target[0] - start[0])
	dy = abs(target[1] - start[1])
	xi = start[0]
	yi = start[1]
	n = 1 + dx + dy
	x_dir = np.sign(target[0] - start[0])
	y_dir = np.sign(target[1] - start[1])
	error = dx - dy;
	dx *= 2
	dy *= 2

	for i in xrange(n):
		if map.grid[xi,yi] is not map.empty and map.grid[xi,yi] > 0:
			return xi, yi

		if error > 0:
			xi += x_dir
			error -= dy
		else:
			yi += y_dir
			error += dx
	return target
