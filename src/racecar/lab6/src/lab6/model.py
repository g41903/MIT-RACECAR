wheelbase = 0.35

import numpy as np
from monte_carlo_localization.particle_filter import pose_dtype

# polygon points, going counter-clockwise
# we assume this is convex, to makes it easier
# homogeneous
bounds = np.array([
	[-0.08, -0.15, 1],
	[-0.08, 0.15, 1],
	[0.42, 0.15, 1],
	[0.42, -0.15, 1],
])

def pose(x, y, theta):
	arr = np.recarray((), dtype=pose_dtype)
	arr.x = x
	arr.y = y
	arr.theta = theta
	return arr

def poses_to_matrices(poses):
	"""
	Takes an nd-array of poses, and produces an nd array of homogeneous tranformation matrices
	"""
	poses = np.asarray(poses, dtype=pose_dtype).view(np.recarray)
	m = np.zeros(poses.shape + (3,3))

	m[...,0,0] = np.cos(poses.theta)
	m[...,0,1] = -np.sin(poses.theta)
	m[...,1,0] = np.sin(poses.theta)
	m[...,1,1] = np.cos(poses.theta)
	m[...,0,2] = poses.x
	m[...,1,2] = poses.y
	m[...,2,2] = 1

	return m

def pose_from_ros(p):
	""" convert a ros message into a pose object """
	from geometry_msgs.msg import PoseStamped, Pose, TransformStamped, Transform
	if isinstance(p, PoseStamped):
		p = p.pose
	elif isinstance(p, TransformStamped):
		p = p.transform

	if isinstance(p, Pose):
		pos = p.position
		quat = p.orientation
	elif isinstance(p, Transform):
		pos = p.translation
		quat = p.rotation
	else:
		raise TypeError

	theta = 2*np.arctan2(quat.z, quat.w)
	return pose(pos.x, pos.y, theta)

def pose_to_ros(msg_type, p):
	from geometry_msgs.msg import Pose, Transform, Point, Vector3, Quaternion
	quat = Quaternion(
		0, 0, np.sin(.5*p.theta),np.cos(.5*p.theta)
	)
	if msg_type == Pose:
		return Pose(
			position=Point(p.x, p.y, 0),
			orientation=quat
		)
	elif msg_type == Transform:
		return Transform(
			translation=Vector3(p.x, p.y, 0),
			rotation=quat
		)
	else:
		raise ValueError("msg_type must be Pose or Transform")


def robot_fits(map, pose, center_only=True):
	"""
	Returns true if the robot at `pose` does not intersect any obstacles in `map`
	If passed an array, only evaluates until the first False, and assumes all
	remaining items are also false

	@param map:  a mcl.Map object
	@param pose: a pose_dtype recarray
	"""
	transform = poses_to_matrices(pose)

	# assume the fill_value is not 0
	grid = map.grid.data

	if center_only:
		center = transform[...,2]
		return grid[map.index_at(center)] == 0
	else:
		# turn car-relative coordinates to absolute map coordinates
		transform_T = transform.swapaxes(-1,-2)
		local_to_index = transform_T.dot(map.inv_transform.T[[0,1,3],:])

		# shape (len(bounds),) + poses.shape + (2,)
		index_bounds = bounds.dot(local_to_index)[...,:-1]

		# shape poses.shape + (len(bounds),) + (2,)
		index_bounds = np.rollaxis(index_bounds, 0, -1)
		# reverse x,y because maps are row-major
		index_bounds = index_bounds[...,::-1]

		res = np.zeros(pose.shape, dtype=np.bool)

		for idx in np.ndindex(res.shape):
			index_bound = index_bounds[idx]

			# find bounds and slice, for speed
			i_min = np.floor(np.min(index_bound[:,0])).astype(np.intp)
			i_max = np.floor(np.max(index_bound[:,0])).astype(np.intp)
			j_min = np.floor(np.min(index_bound[:,1])).astype(np.intp)
			j_max = np.floor(np.max(index_bound[:,1])).astype(np.intp)

			# work with a much smaller object for speed
			submap = grid[i_min:i_max+1,j_min:j_max+1]
			index_bound = index_bound.astype(np.intp) - np.array([i_min, j_min])

			# array of coordinate pairs
			iijj = np.rollaxis(np.indices(submap.shape), 0, 3)

			inside_locs = np.ones(submap.shape, dtype=np.bool)

			for pbi in range(index_bounds.shape[-2]):
				pai = pbi - 1
				pa = index_bound[pai]
				pb = index_bound[pbi]

				inside_locs &= np.cross(iijj - pa, pb - pa) <= 0

			res[idx] = (submap[inside_locs] <= 0).all()

			# quit early if we find a collision
			if not res[idx]:
				break

		return res
