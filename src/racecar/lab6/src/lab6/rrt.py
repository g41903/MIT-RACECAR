from abc import abstractmethod, ABCMeta
from collections import namedtuple
import numpy as np

from . import keyeddict

from .tree import ArrayBackedTree, EdgeTreeNode

class RRT(object):
    """
    Represents an RRT sampling over a type T

    An abstract class that should be overriden to implemented the missing methods
    """
    __metaclass__ = ABCMeta

    class CantExtend(ValueError):
        """ Raised from extend() if such an extension is not possible """
        pass

    class Hooks(object):
        def pre_extend(self, rrt, src, dest): pass
        def post_extend(self, rrt, edge): pass
        def rerooted(self, rrt, old_root, new_root): pass

    class Edge(namedtuple('Edge', 'src dest')):
        @property
        def reversed(self):
            return RRT.Edge(src=self.dest, dest=self.src)

        def __getitem__(self, it):
            if it == np.s_[::-1]:
                return self.reversed
            else:
                return super(RRT.Edge, self).__getitem__(it)

        def at(self, f):
            return self.src + np.asarray(f)[...,np.newaxis]*(self.dest - self.src)

        def interpolate(self, step):
            """ default implementation - ignore argument"""
            return [self.src, self.dest]

    nodes = None

    def __init__(self, origin, goal, hooks=Hooks()):
        """
        @param origin: the start position, of type T
        @param goal:   the end position, of type T
        @param hooks:  callbacks for debugging mid-algorithm
        """
        self.goal = goal
        self.origin = origin
        if self.nodes is None: self.nodes = []
        self._tree = ArrayBackedTree(EdgeTreeNode, self.nodes)
        self._root = self._tree.Node(origin)
        self._hooks = hooks

        self._hooks.rerooted(self, None, origin)

    def get_nearest_node(self, target_node, **kwargs):
        """
        Given a target node, finds the closest node in self.nodes to the target.
        """
        # calculate the distance any any extra data for each node, then find the
        # minimum
        distances, kwargs = self._distance_metric_vec(self.nodes, target_node, **kwargs)
        best = np.argmin(distances)
        return self._tree[best], distances[best], kwargs[best]

    def get_nearest_neighbors(self, target_node, k):
        """
        Given a target node, returns a numpy array of the k closest nodes to the target.
        """
        distances, kwargs = self._distance_metric_vec(self.nodes, target_node)
        nearest = np.argpartition(distances, k)[:k]
        return np.array(self.nodes)[nearest], kwargs[nearest]

    def run_once(self):
        target_node = self._sample()

        closest_node, dist, baton = self.get_nearest_node(target_node)

        # unreachable from any node
        if dist == np.inf:
            return

        self._hooks.pre_extend(self, closest_node.value, target_node)
        try:
            result_edges = self._extend(closest_node.value, target_node, baton)
        except RRT.CantExtend:
            return

        if not getattr(self._extend, 'multi', False):
            result_edges = [result_edges]

        for result_edge in result_edges:
            self._hooks.post_extend(self, result_edge)
            closest_node = closest_node.add_child(
                self._tree.Node(result_edge.dest, edge_value=result_edge)
            )

        return closest_node

    def reroot(self, new_root):
        """ Reroot the tree, so that all edges now point away from the given node """

        new_by_dest = {}

        path = list(self.path_to(new_root))

        self._tree[new_root_id].make_root()

        old_root = self.origin
        self.origin = new_root
        self._hooks.rerooted(self, old_root, new_root)


    def run(self):
        """ Run the RRT until it gets close enough """
        while True:
            res = self.run_once()
            if res is not None and self._is_close_enough(res.value):
                return self.path_to(res)

    def path_to(self, node):
        """ return the sequence of edges to get to a given node in the tree """
        return [
            n.edge_value
            for n in node.path
            if n.edge_value is not None
        ]


    @property
    def edges(self):
        """ Yields Edge(a,b) for each edge in the tree a -> b. For debugging """
        return [
            node.edge_value
            for node in self._tree
            if node.edge_value is not None
        ]

    def prune(self):
        """ Remove all branches which now intersect obstacles """
        it = iter(self._root)
        next(it)  # skip the root
        for node in it:
            # remove all branches that go via an intersecting edge
            if not self._check_free(node.edge_value.interpolate(0.2)).all():
                node.parent = None

        # pack is picky about local variables
        node = None
        del it
        self._tree.pack()

    def _distance_metric_vec(self, srcs, dest, **kwargs):
        """ Vectorized distance metric. Override in sub class if possible """
        dist = np.empty(len(srcs), np.float64)
        data = np.empty(len(srcs), np.object_)
        for i in range(len(srcs)):
            dist[i], data[i] = self._distance_metric(srcs[i], dest, **kwargs)
        return dist, data

    @abstractmethod
    def _sample(self):
        """ returns a random T, to be specified by the subclass """
        raise NotImplementedError

    @abstractmethod
    def _distance_metric(self, src, dest, **kwargs):
        """
        @returns distance, baton

        distance should be a scalar indicating how similar two positions are. It
        should correlate with time taken to get there, not necesarily cartesian
        distance

        baton is an object to be passed onto _extend, for storing calculations
        """
        raise NotImplementedError

    @abstractmethod
    def _extend(self, src, dest, baton):
        """
        Should return a new location by taking some action from src in the directuon of dest

        @raises RRT.CantExtend
        """
        raise NotImplementedError

    @abstractmethod
    def _check_free(self, locations):
        """ Check if locations like in free space. Return an ndarray(bool) """
        raise NotImplementedError

    def _is_close_enough(self, node):
        """
        Return true if node is close enough to the goal

        Default behaviour is false unless exactly equal
        """
        return node == self.goal