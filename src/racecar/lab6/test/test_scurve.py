import unittest
from lab6 import AckermannRRT2, model
import numpy as np

class SCurveTests(unittest.TestCase):
    def check_result(self, d, b):
        print "c1", b.c1.round(2)
        print "c2", b.c2.round(2)
        print "angle1", np.degrees(b.angle1)
        print "angle2", np.degrees(b.angle2)
        print "r", b.r
        print "d", d

        self.assertTrue(np.isfinite(b.angle1).all(), 'angle1 is not finite, {}'.format(b.angle1))
        self.assertTrue(np.isfinite(b.angle2).all(), 'angle2 is not finite, {}'.format(b.angle2))
        self.assertTrue(np.isfinite(b.r).all(), 'r is not finite, {}'.format(b.r))

    def test_simple(self):
        p1 = model.pose(0, 0, np.pi)
        p2 = model.pose(10, 0, 0)
        a = AckermannRRT2(None, p1, p2)

        d, b = a._distance_metric(p1, p2)

        self.check_result(d, b)

    def test_vectorized(self):
        p1 = model.pose(0, 0, np.pi)
        p2 = model.pose(10, 0, 0)
        a = AckermannRRT2(None, p1, p2)

        ps = np.recarray(2, dtype=model.pose_dtype)
        ps[0] = model.pose(0, 0, np.pi)
        ps[1] = model.pose(0, 0, -np.pi)

        d, b = a._distance_metric(ps, p2)

        self.check_result(d, b)

    def test_empty(self):
        p1 = model.pose(0, 0, np.pi)
        p2 = model.pose(10, 0, 0)
        a = AckermannRRT2(None, p1, p2)

        ps = np.recarray(0, dtype=model.pose_dtype)
        d, b = a._distance_metric(ps, p2)

        self.check_result(d, b)

    def test_parallel(self):
        p1 = model.pose(0, 0, 0)
        p2 = model.pose(10, 5, 0)
        a = AckermannRRT2(None, p1, p2)

        d, b = a._distance_metric(p1, p2)

        self.check_result(d, b)

    def test_straight(self):
        p1 = model.pose(0, 0, 0)
        p2 = model.pose(10, 0, 0)
        a = AckermannRRT2(None, p1, p2)

        d, b = a._distance_metric(p1, p2)

        self.check_result(d, b)

    def test_straight_diag(self):
        p1 = model.pose(0, 0, np.pi/4)
        p2 = model.pose(10, 10, np.pi/4)
        a = AckermannRRT2(None, p1, p2)

        d, b = a._distance_metric(p1, p2)

        self.check_result(d, b)

    def test_straightish(self):
        p1 = model.pose(0, 0, 0)
        p2 = model.pose(10, 0, 0.001)
        a = AckermannRRT2(None, p1, p2)

        d, b = a._distance_metric(p1, p2)

        self.check_result(d, b)

    def test_cocircular(self):
        p1 = model.pose(0, 0, -np.pi/4)
        p2 = model.pose(10, 0, np.pi/4)
        a = AckermannRRT2(None, p1, p2)

        d, b = a._distance_metric(p1, p2)

        self.check_result(d, b)

    def test_equilaterateral(self):
        p1 = model.pose(0, 0, -np.pi/6)
        p2 = model.pose(8, 0, -np.pi/6)
        a = AckermannRRT2(None, p1, p2)

        d, b = a._distance_metric(p1, p2)

        self.check_result(d, b)

        p1 = model.pose(0, 0, -np.pi/6)
        p2 = model.pose(10, -2*np.sqrt(3), -np.pi/2)
        a = AckermannRRT2(None, p1, p2)

        d, b = a._distance_metric(p1, p2)

        self.check_result(d, b)
        raise NotImplementedError


if __name__ == '__main__':
    unittest.main(buffer=True)
