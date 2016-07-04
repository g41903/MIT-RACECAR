from collections import MutableSequence
import sys

import numpy as np

class NumpyList(MutableSequence):
    """
    An implementation of an arraylist, that reallocates whenever an append
    would not fit within the existing allocation

    In the interest of preventing memory leaks, the array is resized in place,
    with reference checking.

    This means that there must be no surviving references to the array when it
    is resized! For instance, the following will crash:

        >>> a = NumpyList(np.float)
        >>> a.append(1)
        >>> b = a[:1]
        >>> a.append(2)
        ValueError

    Because b refers to the old array, before it is resized. For this to work,
    either a copy must be made:

        >>> a = NumpyList(np.float)
        >>> a.append(1)
        >>> b = a[:1].copy()
        >>> a.append(2)

    Or the view must be deleted before another append occurs

        >>> a = NumpyList(np.float)
        >>> a.append(1)
        >>> b = a[:1]
        >>> del b
        >>> a.append(2)

    While general insertion is supported, it is inneficient if not at the end
    of the array

    """
    CAPACITY_FACTOR = 4  # the factor to increase the length by when enlarging

    def __init__(self, dtype, atype=np.ndarray, capacity=1):
        self._data = atype(capacity, dtype=dtype)
        self.count = 0

    @property
    def data(self):
        """ provides access to the underlying np array """
        return self._data[:self.count]

    @property
    def capacity(self):
        """ The number of elements the list can hold before needing to resize """
        return self._data.shape[0]

    @capacity.setter
    def capacity(self, value):
        """ Resize the list """
        if value < self.count:
            raise ValueError('Cannot lower capacity beyond data length')
        sys.exc_clear()  # prevent reference leaking in tracebacks
        self._data.resize(value)

    def __repr__(self):
        return '<NumpyList of {!r}>'.format(self.data)

    # methods below are required for MutableSequence abstract base class

    def __len__(self):
        return self.count

    def __getitem__(self, i):
        return self.data[i]

    def __setitem__(self, i, value):
        self.data[i] = value

    def __delitem__(self, i):
        self.data[i:][:-1] = self.data[i:][1:]
        self.count -= 1

    def insert(self, i, value):
        if self.count + 1 > self.capacity:
            self.capacity *= self.CAPACITY_FACTOR

        self.data[i:][1:] = self.data[i:][:-1]
        self.count += 1
        self._data[i] = value

    def pop(self, i=-1):
        val = self[i].copy()
        del self[i]
        return val
