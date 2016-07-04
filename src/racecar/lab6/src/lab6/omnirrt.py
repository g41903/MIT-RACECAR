from . import RRT

import numpy as np

class OmniRRT(RRT):
    """
    See rrt.py for documentation of the below implemented methods

    In this implementation, we choose to ignore the theta sampled by the next point,
    and just accept whatever theta we end up with when we get there
    """
    def __init__(self, map, origin, goal, **kwargs):
        super(OmniRRT, self).__init__(origin, goal, **kwargs)
        self.map = map

    def _sample(self):
        # fit a square bounding box
        span = np.linalg.norm(self.goal - self.origin) * 1.5
        center = (self.goal + self.origin) / 2
        bottom = center - np.ones(2) * span / 2
        top = center + np.ones(2) * span / 2

        if np.random.rand() < 0.05:
            return self.goal
        else:
            return np.random.uniform(bottom, top)

    def _distance_metric(self, src, dest, **kwargs):
        return np.linalg.norm(src - dest), None

    def _extend(self, src, dest, baton):
        MAX_DIST = 0.5

        vec = dest - src
        dist = np.linalg.norm(vec)

        if dist > MAX_DIST:
            vec *= MAX_DIST / dist
            dist = MAX_DIST

        # step along the path. We could also raycast...
        n_steps = dist // self.map.resolution
        fracs = np.linspace(0, 1, n_steps + 1)
        points = src + fracs[:,np.newaxis] * vec

        # find the first bad step along the path
        bad_at = argfirst(~self._check_free(points))

        # no wall at all
        if not np.any(bad_at):
            return self.Edge(src, points[-1])

        if bad_at <= 1:
            raise RRT.CantExtend

        return self.Edge(src, points[bad_at - 1])

    def _check_free(self, points):
        points3 = np.append(points, np.zeros(points.shape[:-1] + (1,)), axis=-1)
        ii, jj = self.map.index_at(points3)
        values = self.map[ii, jj].filled(100)
        return ~(values > 0)

    def _is_close_enough(self, node):
        # ignore theta for determining if we reached the goal
        return np.linalg.norm(node - self.goal) < 0.1
