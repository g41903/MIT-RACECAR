from . import RRT, model
from .numpylist import NumpyList
from .geom import slerp, connect_pose_to_point, argfirst

import numpy as np


class AckermannRRT(RRT):
    """
    See rrt.py for documentation of the below implemented methods

    In this implementation, we choose to ignore the theta sampled by the next point,
    and just accept whatever theta we end up with when we get there
    """

    MIN_RADIUS = 1.0
    MAX_STEP = 1
    MIN_STEP = 0.025

    class Edge(RRT.Edge):
        def __new__(cls, src, dest, center, angle, radius):
            res = super(AckermannRRT.Edge, cls).__new__(cls, src, dest)
            res.center = center
            res.angle = angle
            res.radius = radius
            return res

        @property
        def is_forward(self):
            return self.angle * self.radius >= 0

        @property
        def reversed(self):
            return AckermannRRT.Edge(
                src=self.dest, dest=self.src, center=self.center,
                radius=self.radius, angle=-self.angle)

        def at(self, fracs):
            """
            Return the pose at fraction `fracs` along this path, so at(0) = src, at(1) = dest
            """
            poses = np.recarray(fracs.shape, dtype=model.pose_dtype)
            poses.xy = self.center + slerp(
                self.src.xy - self.center,
                self.dest.xy - self.center,
                np.asarray(fracs)[:,np.newaxis], omega=self.angle)
            poses.theta = self.src.theta + self.angle * fracs
            return poses

        def interpolate(self, step):
            """
            Return poses sampled at intervals of at most step
            """
            arc_dist = abs(self.radius * self.angle)
            n_steps = np.ceil(arc_dist / step)
            fracs = np.linspace(0, 1, n_steps + 1)
            return self.at(fracs)

    def __init__(self, map, origin, goal, allow_reverse=True, **kwargs):
        """
        @param origin:  np.recarray((), pose_dtype)
        @param goal:    np.array of shape (2,)
        """
        self.nodes = NumpyList(dtype=model.pose_dtype, atype=np.recarray)
        super(AckermannRRT, self).__init__(origin, goal, **kwargs)
        self.map = map

        self.allow_reverse = allow_reverse

    def _node_key(self, node):
        return node.data

    def _sample(self):
        initial = self.origin
        rand = np.random.rand()
        if rand < 0.05:
            return self.goal
        elif rand < 0.30:
            initial, _, _ = self.get_nearest_node(self.goal)
            initial = initial.value


        # fit a square bounding box
        span = np.linalg.norm(self.goal - initial.xy) * 1.5
        center = (self.goal + initial.xy) / 2
        bottom = center - np.ones(2) * span / 2
        top = center + np.ones(2) * span / 2

        # try a bunch of points all at once, to save overhead
        CHUNK_SIZE = 30
        attempt = np.empty((CHUNK_SIZE, 3))

        while True:
            attempt[:,:2] = np.random.uniform(bottom, top, size=(CHUNK_SIZE, 2))
            attempt[:,2] = 1

            cell = self.map[self.map.index_at(attempt)]
            ok = ~cell.mask & (cell.data <= 0)
            if ok.any():
                return attempt[ok,:2][0]


    def _distance_metric(self, srcs, dest, exchanged=False):
        if isinstance(srcs, NumpyList):
            srcs = srcs.data

        arc = connect_pose_to_point(srcs, dest)

        # we have a fixed turning radius
        possible = (np.abs(arc.radius) >= self.MIN_RADIUS) | (arc.radius == 0)
        if self.allow_reverse != exchanged:
            possible &= (arc.radius * arc.angle) > 0

        dist = np.where(possible, np.abs(arc.radius * arc.angle), np.inf)

        return dist, arc

    _distance_metric_vec = _distance_metric

    def _extend(self, src, dest, baton):
        b = baton

        # find how far along the arc we are allowed to travel
        arc_dist = abs(b.angle*b.radius)
        if arc_dist < self.MIN_STEP:
            raise RRT.CantExtend
        elif arc_dist >= self.MAX_STEP:
            f_max = self.MAX_STEP / arc_dist
            arc_dist = self.MAX_STEP
        else:
            f_max = 1


        # break the arc up into a number of steps
        car_length = 0.5
        n_steps = arc_dist // max(self.map.resolution, 0.5*car_length)
        fracs = np.linspace(0, f_max, n_steps + 2)[1:]

        assert len(fracs) != 0

        # find the pose at each position along the arc
        poses = np.recarray(fracs.shape, dtype=model.pose_dtype)
        poses.xy = b.center + slerp(b.center_to_src, b.center_to_dest, fracs[:,np.newaxis], omega=b.angle)
        poses.theta = src.theta + b.angle * fracs

        # find the first bad step along the path
        # 0 and 1 mean we already failed or would fail on the first step
        bad_at = argfirst(~self._check_free(poses))
        if bad_at <= 1:
            raise RRT.CantExtend

        # we need to copy src so that we can resize nodes safely, else we have a dangling reference
        return self.Edge(src.copy(), poses[bad_at-1],
            center=b.center, radius=b.radius, angle=fracs[bad_at-1]*b.angle)

    def _is_close_enough(self, node):
        # ignore theta for determining if we reached the goal
        return np.linalg.norm(node.xy - self.goal) < 0.25

    def _check_free(self, poses):
        return model.robot_fits(self.map, poses, center_only=False)
