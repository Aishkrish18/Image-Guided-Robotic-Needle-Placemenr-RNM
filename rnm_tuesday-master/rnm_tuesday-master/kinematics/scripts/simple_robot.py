#!/usr/bin/env python3

import numpy as np
from math import pi, cos, sin


def tf_mat_from_dh(alpha, a, d, angle):
    tf_matrix = np.array([[cos(angle), -sin(angle), 0, a],
                           [sin(angle) * cos(alpha), cos(angle) * cos(alpha), -sin(alpha), -d * sin(alpha)],
                           [sin(angle) * sin(alpha), cos(angle) * sin(alpha), cos(alpha), d * cos(alpha)],
                           [0, 0, 0, 1]])
    # dtype= np.float32
    return tf_matrix


class RobotState:

    def __init__(self):
        self.theta = np.zeros(8)
        self.a = np.array([0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0])
        self.alpha = np.array([0, -pi / 2, pi / 2, pi / 2, -pi / 2, pi / 2, pi / 2, 0])
        self.d = np.array([0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107])
        self.joints_max = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        self.joints_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
        self.vel_max = np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100])
        self.acc_max = np.array([15, 7.5, 10, 12.5, 15, 20, 20])
        self.jerk_max = np.array([7500, 3750, 5000, 6250, 7500, 10000, 10000])

    def callback(self, msg):
        self.theta[:-1] = np.asarray(msg.position, dtype=np.float64)

    def calculate_tf(self, q, end_joint=8):
        q = np.append(q, 0)
        t_mat_end = np.eye(4, 4)
        for i in range(end_joint):
            t_mat_end = t_mat_end @ tf_mat_from_dh(self.alpha[i], self.a[i], self.d[i], q[i])
        return t_mat_end

    def check_limits(self, v, a, j):
        cond_v = np.all(abs(v) < self.vel_max)
        cond_a = np.all(abs(a) < self.acc_max)
        cond_j = np.all(abs(j) < self.jerk_max)
        #if np.any(theta1[:-1] < self.joints_min) or np.any(theta1[:-1] > self.joints_max):
        #    return False
        return cond_v & cond_a & cond_j
