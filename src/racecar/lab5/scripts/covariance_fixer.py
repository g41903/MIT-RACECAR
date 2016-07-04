#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from rospyext import *
import math
import numpy as np

class CovarianceFixerNode(Node):
    fixed = Publisher('~fixed', Odometry)
    @Subscriber('~broken', Odometry)
    def sub_odom(self, odom):
        # recover the variables observable from the C code
        wheelbase_ = 0.25  # from vesc.yaml
        current_speed = odom.twist.twist.linear.x
        current_angular_velocity = odom.twist.twist.angular.z
        if current_speed != 0:
            current_steering_angle = math.atan(wheelbase_ * current_angular_velocity / current_speed)
        else:
            current_steering_angle = 0 # this isn't needed in the C code
        M_PI = math.pi
        cos = math.cos
        tan = math.tan
        pow = math.pow

        # below comes from C code
        # current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;

        # calculate the stuff
        # "var" stands for variance
        current_speed_var = pow(0.2, 2);
        current_steering_angle_var = pow(30 * M_PI / 180, 2);  # two degrees

        # Using a taylor expansion:
        # Var[tan(csa)] ~= Var[tan(E[csa]) + 1 / cos^2(E[csa]) * (csa - E[csa])]
        #                = Var[csa] / cos^2(E[csa])
        tan_csa_var = current_steering_angle_var / (cos(current_steering_angle) * cos(current_steering_angle));

        # By expanding expectations:
        #    Var[current_speed * tan(current_steering_angle) / wheelbase_]
        #  = Var[current_speed * tan(current_steering_angle)] / wheelbase_
        #  = Var[current_speed] * Var[tan_csa] + Var[current_speed]*E[tan_csa]^2 + Var[tan_csa]*E[current_speed]^2
        current_angular_velocity_var = (
            current_speed_var * tan_csa_var
            + current_speed_var * tan(current_steering_angle) * tan(current_steering_angle)
            + tan_csa_var * current_speed * current_speed
        ) / wheelbase_;

        # TODO
        # Cov[current_speed, cav] = E[cs * cav] - E[cs]*E[cav]
        #                         = E[cs * cs * tan(csa) / wheelbase_] - E[cs]*E[cav]
        #                         = E[cs^2] * E[tan(csa)] / wheelbase_ - E[cs]*E[cav]
        #                         = (Var[cs] + E[cs]^2) * E[tan(csa)] / wheelbase_ - E[cs]*E[cav]
        # covar = (current_speed_var + current_speed*current_speed) * tan(current_steering_angle) / wheelbase_ - current_speed * current_angular_velocity
        covar = current_speed_var * current_angular_velocity

        # patch the messaeg
        odom.twist.covariance = np.asarray(odom.twist.covariance).reshape(6,6)
        odom.twist.covariance[0,0] = current_speed_var;
        odom.twist.covariance[5,5] = current_angular_velocity_var;
        odom.twist.covariance[0,5] = covar;
        odom.twist.covariance[5,0] = covar;
        odom.twist.covariance = tuple(odom.twist.covariance.ravel())
        self.fixed.publish(odom)

        if abs(current_angular_velocity) > 1e-7:
            rospy.logdebug('{}, {:.10f} | {}, {:.10f}'.format(current_speed, current_angular_velocity, current_angular_velocity_var, covar))

if __name__ == '__main__':
    rospy.init_node("covariance_fixer")
    node = CovarianceFixerNode()
    rospy.spin()
