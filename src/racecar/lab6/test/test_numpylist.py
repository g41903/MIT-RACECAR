import unittest
import sys

import numpy as np

from lab6.model import pose_dtype, pose
from lab6 import NumpyList


class NumpyListTests(unittest.TestCase):
    def test_with_recarray(self):
        ll = NumpyList(dtype=pose_dtype, atype=np.recarray)
        ll.append(pose(1, 2, 3))
        ll.append(pose(4, 5, 6))
        ll.append(pose(7, 8, 9))

        self.assertEqual(ll[0], pose(1, 2, 3), "element lookup works")
        self.assertEqual(ll[1].x, 4, "recarrays class is preserved")

        ll.capacity = 100

        with self.assertRaises(ValueError):
            ll.capacity = 0

        # check that pops work
        self.assertEqual(ll.pop(0), pose(1, 2, 3))
        self.assertEqual(ll.pop(-1), pose(7, 8, 9))
        self.assertEqual(ll[0], pose(4, 5, 6))


    def test_resize_after_indexerror_recarray(self):
        ll = NumpyList(dtype=pose_dtype, atype=np.recarray)
        ll.append(pose(1, 2, 3))
        ll.append(pose(4, 5, 6))

        self.assertEqual(sys.getrefcount(ll._data), 2)

        with self.assertRaises(IndexError, msg="indexerrors are raised correctly"):
            ll[2]

        ll.capacity = 100

        self.assertEqual(sys.getrefcount(ll._data), 2)


if __name__ == '__main__':
    unittest.main()
