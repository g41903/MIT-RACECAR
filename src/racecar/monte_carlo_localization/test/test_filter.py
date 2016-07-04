import unittest
import numpy as np
from monte_carlo_localization.particle_filter import state_update, measurement_update


class TestFilter(unittest.TestCase):

    def test_gaussian(self):
        """ Test the filter on some simple gaussians """

        def test_state_update(command, particle):
            return [particle + command + np.random.multivariate_normal([0, 0], np.identity(2))]

        def test_measurement_update( measurement, particle):
            covar = np.matrix(np.identity(len(particle)))
            return (
                1/np.power(2*np.pi, len(particle)/2.0) *
                np.exp(-.5*(measurement - particle).dot(covar.I).dot(measurement-particle))
            )

        # seed the random number generator, for some determinism
        np.random.seed(0)

        particles = [np.array([0, 0])]*100
        loc = np.array([0.0, 0.0])
        command = np.array([5.0, 10.0])

        means = []
        for i in range(10):
            loc += np.random.multivariate_normal(command, np.identity(2))
            measurement = np.random.multivariate_normal(loc, np.identity(2))
            M = len(particles)
            particles = state_update(particles, lambda p: test_state_update(command, p))
            particles = measurement_update(particles, lambda p: test_measurement_update(measurement, p), M)
            mean = np.mean(particles, axis=0)
            means.append(mean)

        means = np.array(means)
        expected = command * (np.arange(10)[:,np.newaxis] + 1)

        print "means:"
        print means
        print "expected:"
        print expected

        np.testing.assert_allclose(means, expected, atol=7)

if __name__ == '__main__':
    unittest.main()
