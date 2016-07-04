from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, TwistWithCovariance, Twist, Vector3
import numpy as np

def odom_motion_update(odom, particles, d_t, multiply_factor):
    new_particles = np.recarray((multiply_factor,) + particles.shape, dtype=particles.dtype)

    mask_2d = np.array([True, False, False, False, False, True])
    covar = np.matrix(odom.twist.covariance).reshape(6,6)[mask_2d][:,mask_2d]

    samples = np.random.multivariate_normal(
        [odom.twist.twist.angular.z, odom.twist.twist.linear.x],
        covar,
        new_particles.shape
    )
    w_zs, v_xs = samples[...,0], samples[...,1]

    d_thetas = w_zs * d_t
    d_dist = v_xs * d_t

    new_particles.x = particles.x + d_dist*np.cos(particles.theta + d_thetas)
    new_particles.y = particles.y + d_dist*np.sin(particles.theta + d_thetas)
    new_particles.theta = particles.theta + d_thetas

    # flatten back into a list
    new_particles = new_particles.reshape(-1)

    return new_particles
