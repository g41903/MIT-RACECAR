import numpy as np
from nav_msgs.msg import OccupancyGrid
import tf.transformations as transformations
import ros_numpy
import itertools

occupancy_weight = 12

class Map(object):
	empty = np.ma.masked

	def __init__(self, ros_map=None, map_info=None, frame=None, grid=None):
		# for backwards compatibility, accept a OccupancyGrid
		if ros_map is not None:
			assert isinstance(ros_map, (OccupancyGrid, ros_numpy.numpy_msg(OccupancyGrid)))

			if map_info is not None or frame is not None or grid is not None:
				raise ValueError('If ros_map is specified, it must be the only argument')

			map_info = ros_map.info
			frame = ros_map.header.frame_id
			grid = ros_numpy.numpify(ros_map)
		else:
			assert map_info is not None
			assert frame is not None
			assert grid is not None

		self.info = map_info
		self.frame = frame
		self.grid = grid

		self.resolution = map_info.resolution

		self.pos = map_info.origin.position
		self.orient = map_info.origin.orientation

		# transform from index to world
		self.transform = np.dot(
			transformations.translation_matrix([self.pos.x, self.pos.y, self.pos.z]),
			transformations.quaternion_matrix([self.orient.x, self.orient.y, self.orient.z, self.orient.w])
		).dot(
			transformations.scale_matrix(map_info.resolution)
		)

		idxs = [0, 1, 3]
		self.inv_transform = np.linalg.inv(self.transform)[idxs,:]
		self.transform = self.transform[:,idxs]

	def __getitem__(self, key):
		"""
		Look up a cell in the map, but clip to the edges

		For instance, `map[-1, -1] == map[0, 0]`, unlike in a normal np array
		where it would be `map[map.shape[0]-1, map.shape[1]-1]`

		Note that this only applies for pairwise integer indexing. Indexing
		with boolean masks or slice objects uses the normal indexing rules.
		"""
		if not isinstance(key, tuple):
			# probably a mask?
			return self.grid[key]

		if len(key) != 2:
			# row, column, or just wrong
			return self.grid[key]

		if any(np.min_scalar_type(i) == np.bool for i in key):
			# partial mask
			return self.grid[key]

		if any(isinstance(i, slice) for i in key):
			# normal slicing
			return self.grid[key]

		keys = np.ravel_multi_index(key, dims=self.grid.shape, mode='clip')

		# workaround for https://github.com/numpy/numpy/pull/7586
		if keys.ndim == 0:
			return self.grid.take(keys[np.newaxis])[0]
		else:
			return self.grid.take(keys)

	def merge(self, other_map):
		assert isinstance(other_map, Map)
		assert self.resolution == other_map.resolution
		assert self.frame == other_map.frame
		assert self.orient == other_map.orient

		mask = other_map.grid > 0
		other_origin = self.index_at(other_map.pos_at(0,0))
		i, j = np.indices(other_map.grid.shape)[:,mask]
		my_i = i + other_origin[0]
		my_j = j + other_origin[1]
		self.grid[my_i,my_j] = other_map.grid[i,j]

	def index_at(self, pos, world_to_map=None):
		"""
		return i, j such that self.grid[i,j] represents the point(s) pos

		optionally takes a 4x4 matrix as a kwarg to convert `pos` into the map
		frame, which is more efficient to do at the same time as the
		other tranformation. So:

			index_at(pos, world_to_map=world_to_map)

		Is a more efficient form of

			index_at(pos.dot(world_to_map.T))

		"""
		pos = np.asarray(pos)
		# append a 1 to the last axis
		if pos.shape[-1] != 4:
			pos = np.append(pos, np.ones(np.shape(pos)[:-1] + (1,)), axis=-1)

		transform = self.inv_transform
		if world_to_map is not None:
			transform = np.dot(transform, world_to_map)

		xyz_ = np.floor(pos.dot(transform.T)).astype(np.intp)

		return xyz_[...,1], xyz_[...,0]

	def pos_at(self, i, j, map_to_world=None):
		"""
		return the positions, an array of 3-vectors, that corresponds to the
		data at self.grid[i, j]. Normal index broadcasting rules apply.

		optionally takes a 4x4 matrix as a kwarg to convert the result out of
		the map frame, which is more efficient to do at the same time as the
		other tranformation. So:

			pos_at(i, j, map_to_world=map_to_world)

		Is a more efficient form of

			pos_at(i, j).dot(map_to_world.T)

		"""
		b = np.broadcast(i,j)
		if b.shape == ():
			# for speed
			index_vec = [j + 0.5, i + 0.5, 1]
		else:
			index_vec = np.empty(
				b.shape + (3,)
			)
			index_vec[...,0] = i + 0.5
			index_vec[...,1] = j + 0.5
			index_vec[...,2] = 1

		transform = self.transform
		if map_to_world is not None:
			transform = np.dot(map_to_world, transform)

		return np.dot(index_vec, transform.T)[...,:-1]


class HybridMap(Map):
	def __init__(self, *args, **kwargs):
		super(HybridMap, self).__init__(*args, **kwargs)

		# grid might be immutable, so use it as our ground truth
		self.base = self.grid

		# and make a copy for live adjustments
		self.grid = self.grid.copy()
		self.last_slice = np.s_[0:0,0:0]
                self.last_map_update = None

	def merge_transient(self, other_map):
		"""
		Like merge, but only makes a transient change
		"""
		assert isinstance(other_map, Map)
		assert self.resolution == other_map.resolution
		assert self.frame == other_map.frame
		assert self.orient == other_map.orient

		# Add the new map update to our map.

		# apply the local map
		mask = other_map.grid > 0
		i0, j0 = self.index_at(other_map.pos_at(0,0))
		i, j = np.indices(other_map.grid.shape)[:,mask]
		self.grid[i + i0,j + j0] += occupancy_weight # All occupied cells are now only slightly occupied

		# Check to see if we should make a laserscan diff
		if self.last_map_update:
			# Subtract the old laserscan data
			mask = self.last_map_update.grid > 0
			i0, j0 = self.index_at(self.last_map_update.pos_at(0,0))
			i, j = np.indices(self.last_map_update.grid.shape)[:,mask]
			self.grid[i + i0,j + j0] -= occupancy_weight # All occupied cells are now only slightly occupied

		# Save the old laserscan data
		self.last_map_update = other_map
