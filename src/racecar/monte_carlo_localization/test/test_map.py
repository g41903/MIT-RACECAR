import unittest
from monte_carlo_localization import Map
from nav_msgs.msg import OccupancyGrid
import numpy as np


class TestMap(unittest.TestCase):
	@classmethod
	def setUpClass(cls):

		print "Loading bag file"
		import rosbag
		with rosbag.Bag('sample-map.bag') as bag:
			_, cls.map_msg, _ = next(bag.read_messages())
		print "Loaded"

		# WTF, rosbag
		cls.map_msg.__class__ = OccupancyGrid

		cls.m = Map(cls.map_msg)
		cls.d = cls.m.resolution / 2

	def test_pos_from_index(self):
		# indices refer to the center of cells
		self.assertEqual(self.m.index_at([self.d, self.d, 0]), (2000, 2000))

	def test_pos_from_index_multi(self):
		locs = np.array([
			[self.d, self.d, 0],
			[self.d, self.d, 0],
			[self.d, self.d, 0],
			[self.d, self.d, 0],
			[self.d, self.d, 0]
		])
		res = self.m.index_at(locs)
		self.assertIsInstance(res, tuple)
		np.testing.assert_array_equal(res[0], [2000]*5)
		np.testing.assert_array_equal(res[1], [2000]*5)

	def test_index_from_pos(self):
		np.testing.assert_allclose(self.m.pos_at(2000, 2000), [self.d, self.d, 0], atol=self.d/10)

	def test_index_from_pos_multi(self):
		np.testing.assert_allclose(self.m.pos_at(np.array([2000]*5), np.array([2000]*5)), [[self.d, self.d, 0]]*5, atol=self.d/10)


if __name__ == '__main__':
	unittest.main()
