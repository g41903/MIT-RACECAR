import unittest
import monte_carlo_localization as mcl
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import LaserScan
import numpy as np
import ros_numpy
import time



class TestSensorUpdate(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # build a square room
        map_data = np.zeros((400, 400), dtype=np.int8)
        map_data[...] = -1
        map_data[150:251, 150:251] = 100
        map_data[151:250, 151:250] = 0
        msg = ros_numpy.msgify(OccupancyGrid, map_data)
        msg.info.resolution = 0.1
        msg.info.origin = Pose(position=Point(x=-20, y=-20))

        cls.map = mcl.Map(msg)

    def test_map_integrity(self):
        # check the walls are where we wanted them
        locations = np.array([
            [0, 0,  0],
            [0, -5, 0],
            [0, 5,  0],
            [-5, 0, 0],
            [ 5, 0, 0]
        ])
        expected = [0, 100, 100, 100, 100]

        np.testing.assert_array_equal(self.map.grid[self.map.index_at(locations)], expected)


    def _test(self, N_scan, N_part):
        # build a laser scan of a square dead end
        scan = LaserScan(
            angle_min=-np.pi/2,
            angle_increment=np.pi/(N_scan - 1),
            range_max=10,
            range_min=0
        )
        angles = scan.angle_min + np.arange(N_scan)*scan.angle_increment
        scan.ranges = 5 * np.where(np.abs(angles) < np.pi/4, 1/np.cos(angles), 1/np.abs(np.sin(angles)))

        # now create some particles:
        particles = np.recarray(N_part + 1, dtype=mcl.particle_filter.pose_dtype)
        particles[:-1].x = np.random.normal(loc=0, scale=1, size=N_part)
        particles[:-1].y = np.random.normal(loc=0, scale=1, size=N_part)
        particles[:-1].theta = np.random.normal(loc=0, scale=np.pi/8, size=N_part)

        particles[-1] = (0, 0, 0)

        start = time.time()
        print "Started sensor matching"
        weights = mcl.ros_sensor_update.sensor_update(self.map, scan, particles, np.eye(4))
        print 'Took', time.time() - start

        if len(weights) < 20:
            print weights

        self.assertEqual(np.argmax(weights), len(particles) - 1, "The most likely particle should be at the origin")

    def test_1_one_part(self): self._test(N_scan=5, N_part=1)

    def test_2_one_part_large(self): self._test(N_scan=100, N_part=1)

    def test_3_many_part_large(self): self._test(N_scan=100, N_part=20)


if __name__ == '__main__':
    unittest.main()
