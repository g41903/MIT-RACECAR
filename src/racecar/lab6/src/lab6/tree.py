import weakref

class TreeNode(object):
    """ Represents a tree with values on the nodes """
    def __init__(self, value, children=None):
        self.value = value

        self._parent = None
        self._children = frozenset()

        if children is None: children = set()
        self.children = children

    @property
    def parent(self):
        return self._parent

    @parent.setter
    def parent(self, new_parent):
        if self._parent is not None:
            self._parent._children -= {self}
        if new_parent:
            new_parent._children |= {self}
        self._parent = new_parent

    @property
    def children(self):
        return self._children

    @children.setter
    def children(self, new_children):
        for c in new_children:
            c.parent = self

    def add_child(self, node):
        node.parent = self
        return node

    @property
    def descendants(self):
        res = set()
        curr = set(self.children)

        while curr:
            res |= curr
            curr = {child for c in curr for child in c.children}
        return res

    @property
    def path(self):
        at = self
        res = []
        while at is not None:
            res.insert(0, at)
            at = at.parent
        return res

    def make_root(self):
        path = self.path

        for parent, child in zip(path, path[1:]):
            child.children |= {parent}

        self.parent = None

    def __repr__(self):
        if self.children:
            return "{}({!r}, children={!r})".format(self.__class__.__name__, self.value, self.children)
        else:
            return "{}({!r})".format(self.__class__.__name__, self.value)

    def pprint(self, indent=''):
        print '{}{!r}'.format(indent, self.value)
        for c in self.children:
            c.pprint(indent=indent + '  ')

    def __iter__(self):
        return TreeIter(self)


class EdgeTreeNode(TreeNode):
    """ Represents a tree with values on the nodes and edges """
    def __init__(self, value, edge_value=None, children=None):
        if children is None: children = {}
        self._edge_value = edge_value
        super(EdgeTreeNode, self).__init__(value, children)

    @property
    def edge_value(self):
        if self.parent:
            return self._edge_value
        else:
            return None

    @edge_value.setter
    def edge_value(self, value):
        if not self.parent:
            raise ValueError("Root nodes have no associated edge")
        else:
            self._edge_value = value


    def pprint(self, indent=''):
        print '{}+-({!r})--{!r}'.format(indent, self.edge_value, self.value)
        for c in self.children:
            c.pprint(indent=indent + '  ')

    @property
    def parent(self):
        return self._parent

    @parent.setter
    def parent(self, new_parent):
        super(EdgeTreeNode, self.__class__).parent.fset(self, new_parent)
        if not self._parent:
            self._edge_value = None

    @staticmethod
    def _reverse(edge_data):
        try:
            return edge_data[::-1]
        except:
            return edge_data

    def make_root(self):
        """ Reshuffle the tree so that this node is the root """
        path = self.path

        for parent, child in zip(path, path[1:]):
            child.children |= {parent}
            parent.edge_value = self._reverse(child.edge_value)

        self.parent = None

class ArrayBackedMixin(object):
    _value = None

    @property
    def value(self):
        return self._manager._arr[self._value]

    @value.setter
    def value(self, value):
        if self._value is None:
            self._value = len(self._manager._arr)
            self._manager._node_arr.append(weakref.ref(self))
            self._manager._arr.append(value)
        else:
            self._manager._arr[self._value] = value

    def __del__(self):
        if self._manager._default is not self._manager.NO_DEFAULT:
            self.value = self._manager._default

class ArrayBackedTree(object):
    """
    A class that allows a tree to have its nodes back by a single array

    Works well in conjunction with NumpyList

    Exposes a .Node type specific to the instance
    """
    NO_DEFAULT = object()

    def __init__(self, node_type, arr, default=NO_DEFAULT):
        self._node_arr = []
        self._arr = arr
        self._default = default
        self.Node = type(
            '{}.Node'.format(self),
            (ArrayBackedMixin, node_type),
            dict(_manager=self)
        )

    def __getitem__(self, index):
        res = self._node_arr[index]()
        if res is None:
            raise IndexError
        return res

    def __iter__(self):
        for n in self._node_arr:
            res = n()
            if res is not None:
                yield res

    def pack(self):
        """ Remove null entries from this list """
        import gc
        gc.collect()
        refs = [ref for ref in self._node_arr]

        vals = [ref() for ref in refs]
        bad = [val is None for val in vals]
        good_vals = [val for b, val in zip(bad, vals) if not b]
        good_refs = [ref for b, ref in zip(bad, refs) if not b]

        # renumber all the good nodes
        for i, v in enumerate(good_vals):
            self._arr[i] = v.value
            v._value = i

        self._node_arr = good_refs

class TreeIter(object):
    """
    Depth-first iteration through a tree. Provides a method to skip entire
    branches
    """
    def __init__(self, node):
        self.iter_stack = [iter({node})]

    def __iter__(self):
        return self

    def __next__(self):
        while True:
            if not self.iter_stack:
                raise StopIteration
            try:
                at = next(self.iter_stack[-1])
            except StopIteration:
                self.iter_stack.pop()
            else:
                self.iter_stack.append(iter(at.children))
                return at

    next = __next__

    def skip_children(self):
        """ move onto the next sibling for the next value """
        self.iter_stack.pop()
