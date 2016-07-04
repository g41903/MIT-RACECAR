import numpy as np
import operator

class ParticleDistribution(object):
    """
    A random variable internally represented by a particle distribution. Can be
    used in arithmetic expressions, and the particles will update accordingly:

        >>> p = ParticleDistribution([1, 2])
        >>> p + 2
        ParticleDistribution([3, 4], [0.5, 0.5], normalized=True)
        >>> p + p
        ParticleDistribution([2, 3, 3, 4], [0.25, 0.25, 0.25, 0.25], normalized=True)
        >>> (p + p).mean
        3

    """
    val_type = np.object_

    def __init__(self, vals, weights=None, normalized=False):
        """
        Construct a particle distribution given a set of particles and weights.

        If weights is omitted, the particles are assume to have a uniform distribution
        If weights are specified, then `normalized` tells the class that they already sum to 1
        """
        if weights is None:
            weights = np.ones(len(vals)) / len(vals)
            self.normalized = True
        else:
            self.normalized = normalized

        self.particles = np.empty(len(vals), dtype=[('w', np.float64), ('val', self.val_type)])
        self.particles['val'] = vals
        self.particles['w'] = weights

    def normalize(self):
        """ Renormalize the weights. Only needs to be called if using the internal particles array """
        if not self.normalized:
            self.particles['w'] /= np.sum(self.particles['w'])
            self.normalized = True

    def expectation(self, f):
        """ Calculate the expectation of the function f """
        self.normalize()
        return np.sum(self.particles['w'] * f(self.particles['val']))

    def map(self, f):
        """ Apply a function to the values of the particles """
        return ParticleDistribution(f(self.particles['val']), self.particles['weights'], normalized=self.normalized)

    def resample(self, N):
        """ resample the distribution, resetting all the weights to 1 """
        which = np.random.choice(len(self.particles), size=N, p=self.particles['w'])
        return ParticleDistribution(self.particles[which]['val'])

    # convenience expectations
    @property
    def mean(self):
        return self.expectation(lambda x: x)

    @property
    def variance(self):
        return self.expectation(lambda x: x*x) - self.mean^2

    # operator overloads - no numpy_ufunc here, because that's hard
    def __add__(self, other): return self.__op(other, operator.add)
    def __sub__(self, other): return self.__op(other, operator.sub)
    def __mul__(self, other): return self.__op(other, operator.mul)
    def __div__(self, other): return self.__op(other, operator.div)
    def __pow__(self, other): return self.__op(other, operator.pow)
    def __op(self, other, op):
        if isinstance(other, ParticleDistribution):
            weights = (self.particles['w'] * other.particles['w'][:,np.newaxis]).ravel()
            vals = op(self.particles['val'], other.particles['val'][:,np.newaxis]).ravel()
            return ParticleDistribution(vals, weights, normalized=self.normalized and other.normalized)
        else:
            return ParticleDistribution(op(self.particles['val'], other), self.particles['weights'], normalized=self.normalized)

    # a verbose repr
    def __repr__(self):
        return "ParticleDistribution({!r}, {!r}, normalized={!r})".format(self.particles['val'], self.particles['w'], self.normalized)
