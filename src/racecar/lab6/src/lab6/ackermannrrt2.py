from . import RRT, model, AckermannRRT
from .numpylist import NumpyList
from .geom import slerp, connect_pose_to_pose, argfirst

import numpy as np

class AckermannRRT2(RRT):
    """
    See rrt.py for documentation of the below implemented methods

    In this implementation, we choose to ignore the theta sampled by the next point,
    and just accept whatever theta we end up with when we get there
    """
    MIN_RADIUS = 1.0
    MAX_STEP = 1
    MIN_STEP = 0.025

    Edge = AckermannRRT.Edge

    def __init__(self, map, origin, goal, allow_reverse=True, **kwargs):
        """
        @param origin:  np.recarray((), pose_dtype)
        @param goal:    np.array of shape (2,)
        """
        self.nodes = NumpyList(dtype=model.pose_dtype, atype=np.recarray)
        super(AckermannRRT2, self).__init__(origin, goal, **kwargs)
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
        span = np.linalg.norm(self.goal.xy - initial.xy) * 1.5
        center = (self.goal.xy + initial.xy) / 2
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
                p = np.recarray((), dtype=model.pose_dtype)
                p.xy = attempt[ok,:2][0]
                p.theta = np.random.uniform(0, np.pi*2)
                return p

    def _distance_metric(self, srcs, dest):
        if isinstance(srcs, NumpyList):
            srcs = srcs.data

        curve = connect_pose_to_pose(srcs, dest)

        # we have a fixed turning radius
        possible = np.abs(curve.r) >= self.MIN_RADIUS
        if not self.allow_reverse:
            possible &= (curve.r * curve.angle1) > 0
            possible &= (curve.r * curve.angle2) < 0

        # catch instabilities
        possible &= np.isfinite(curve.angle1) & np.isfinite(curve.angle2) & np.isfinite(curve.r)
        dist = np.where(possible,
            np.abs(curve.r * curve.angle1) + np.abs(curve.r * curve.angle2),
            np.inf
        )

        return dist, curve

    _distance_metric_vec = _distance_metric

    def _extend(self, src, dest, baton):
        """
        A dumb extend function without collision detection, to check if the baton is correct
        """
        b = baton
        midpoint = np.recarray((), dtype=model.pose_dtype)
        midpoint.xy = b.midpoint
        midpoint.theta = src.theta + b.angle1

        src = src.copy()

        # convert arrays to scalars, which are somehow mutableish?
        dest = dest[()]
        midpoint = midpoint[()]

        return [
            self.Edge(src, midpoint,
                        center=b.c1, radius=b.r, angle=b.angle1),
            self.Edge(midpoint, dest,
                        center=b.c2, radius=-b.r, angle=b.angle2)
        ]

    def _extend(self, src, dest, baton):
        b = baton

        # truncate the path if we need to
        arc1_dist = abs(b.angle1 * b.r)
        arc2_dist = abs(b.angle2 * b.r)
        if arc1_dist + arc2_dist < self.MIN_STEP:
            raise RRT.CantExtend
        elif arc1_dist >= self.MAX_STEP:
            # first arc is too long
            f1_max = self.MAX_STEP / arc1_dist
            f2_max = 0
            arc1_dist = self.MAX_STEP
            arc2_dist = 0
        elif arc1_dist + arc2_dist < self.MAX_STEP:
            # second arc is too long
            f1_max = 1
            f2_max = (self.MAX_STEP - arc1_dist) / arc2_dist
            arc2_dist = self.MAX_STEP - arc1_dist
        else:
            f1_max = 1
            f2_max = 1

        # break the arc up into a number of steps
        car_length = 0.5
        n1_steps = arc1_dist // max(self.map.resolution, 0.5*car_length)
        fracs1 = np.linspace(0, f1_max, n1_steps + 2)[1:]
        n2_steps = arc2_dist // max(self.map.resolution, 0.5*car_length)
        fracs2 = np.linspace(0, f2_max, n1_steps + 2)[1:]

        # build a list of edges to return
        edges = []

        # we must maintain that either midpoint === src, or midpoint === edges[0].dest
        # exact equality is required, floating point inaccuracies are not acceptable
        midpoint = np.recarray((), dtype=model.pose_dtype)
        midpoint.xy = src.xy
        midpoint.theta = src.theta

        if arc1_dist > 0:
            # find the pose at each position along the first arc
            poses1 = np.recarray(fracs1.shape, dtype=model.pose_dtype)
            poses1.xy = b.c1 + slerp(
                src.xy - b.c1, b.midpoint - b.c1,
                fracs1[:,np.newaxis],
                omega=b.angle1
            )
            poses1.theta = src.theta + b.angle1 * fracs1

            # find the first bad step along the path
            bad_at = argfirst(~model.robot_fits(self.map, poses1, center_only=False))

            # entire arc is clear
            if bad_at == len(poses1) and arc2_dist > 0:
                # we need to make sure we use exactly the same midpoint here
                midpoint.xy = b.midpoint
                midpoint.theta = src.theta + b.angle1
                edges.append(
                    self.Edge(src.copy(), midpoint[()],
                        center=b.c1, radius=b.r, angle=b.angle1)
                )
            else:
                # skip the second arc
                arc2_dist = 0
                if bad_at > 1:
                    edges.append(
                        self.Edge(src.copy(), poses1[bad_at-1],
                            center=b.c1, radius=b.r, angle=fracs1[bad_at-1]*b.angle1)
                    )

        if arc2_dist > 0:
            # find the pose at each position along the second arc
            poses2 = np.recarray(fracs2.shape, dtype=model.pose_dtype)
            poses2.xy = b.c2 + slerp(
                b.midpoint - b.c2, dest.xy - b.c2,
                fracs2[:,np.newaxis],
                omega=b.angle2
            )
            poses2.theta = src.theta + b.angle1 + b.angle2 * fracs2

            # find the first bad step along the path
            bad_at = argfirst(~model.robot_fits(self.map, poses2, center_only=False))
            if bad_at > 1:
                edges.append(
                    self.Edge(midpoint[()], poses2[bad_at-1],
                        center=b.c2, radius=-b.r, angle=fracs2[bad_at-1]*b.angle2)
                )

        if not edges:
            raise RRT.CantExtend

        return edges

    _extend.multi = True

    def _check_free(self, poses):
        return model.robot_fits(self.map, poses, center_only=False)

    def _is_close_enough(self, node):
        # ignore theta for determining if we reached the goal
        dp = np.linalg.norm(node.xy - self.goal.xy)
        dtheta = node.theta - self.goal.theta
        while dtheta > np.pi:
            dtheta -= 2*np.pi
        while dtheta < -np.pi:
            dtheta += 2*np.pi
        return dp < 0.25 and np.abs(dtheta) < np.pi/8
