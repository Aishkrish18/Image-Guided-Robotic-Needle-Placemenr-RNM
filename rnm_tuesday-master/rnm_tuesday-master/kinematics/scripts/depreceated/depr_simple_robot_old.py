#!/usr/bin/env python3
# by cenkt

from math import pi, cos, sin

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# if you need to check the implementation
# import tf.transformations as t


class Robot:
    def __init__(self):
        self.theta = np.zeros(7)
        self.a = np.array([0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0])
        self.alpha = np.array([0, -pi / 2, pi / 2, pi / 2, -pi / 2, pi / 2, pi / 2, 0])
        self.d = np.array([0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107])
        self.joints_max = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        self.joints_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
        self.vel_max = np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100])
        self.acc_max = np.array([15, 7.5, 10, 12.5, 15, 20, 20])
        self.jerk_max = np.array([7500, 3750, 5000, 6250, 7500, 10000, 10000])

    def callback(self, msg):
        self.theta = np.asarray(msg.position, dtype=np.float64)
        # print(self.theta.shape)

    def calculate_tf(self, q, end_eff=None):
        # 45 degrees is to match the x_e with the guiding part of the robot, which is a good cue for camera rotation
        q = np.append(q, 45 * np.pi / 180)
        t_mat_end = np.eye(4, 4)
        #print('lenof q', len(q))
        for i in range(len(q)):
            t_mat_end = t_mat_end @ self.tf_mat_from_dh(self.alpha[i], self.a[i], self.d[i], q[i])
        #print('before end eff')
        #print(np.round(t_mat_end, 4))
        if end_eff == 'cam':
        #    print("Using camera as end_eff")
            t_end_eff = self.tf_mat_from_dh(np.pi / 2, 0, 0, 0)
        elif end_eff == 'needle':
        #    print("using needle as end_eff")
            t_end_eff = self.tf_mat_from_dh(0, 0, 0.15, 0)
        else:
        #    print("using no end_eff")
            t_end_eff = np.eye(4, 4)
        t_mat_end = t_mat_end @ t_end_eff
        return t_mat_end

    def check_limits(self, v, a, j, gas):
        cond_v = np.all(abs(v) < self.vel_max * gas)
        cond_a = np.all(abs(a) < self.acc_max * gas)
        cond_j = np.all(abs(j) < self.jerk_max * gas)
        return cond_v & cond_a & cond_j

    def check_joints(self, q):
        return (self.joints_min < q) & (q < self.joints_max)

    @staticmethod
    def tf_mat_from_dh(alpha, a, d, angle):
        tf_matrix = np.array([[cos(angle), -sin(angle), 0, a],
                              [sin(angle) * cos(alpha), cos(angle) * cos(alpha), -sin(alpha), -d * sin(alpha)],
                              [sin(angle) * sin(alpha), cos(angle) * sin(alpha), cos(alpha), d * cos(alpha)],
                              [0, 0, 0, 1]])
        return tf_matrix


if __name__ == "__main__":
    robot = Robot()
    rospy.init_node('fk_node')
    panda_subscriber = rospy.Subscriber('/joint_states', JointState, robot.callback)
    # end_pose_publisher = rospy.Publisher('~end_effector_pose', Float64MultiArray, queue_size=10)

    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        print(robot.theta)
        end_eff = rospy.get_param('/end_effector', None)
        a = np.round(robot.calculate_tf(robot.theta, end_eff), 4)
        print(a)
        # end_pose_publisher.publish(Float64MultiArray(data=robot.calculate_tf(robot.theta).reshape(-1)))
        # np.set_printoptions(suppress=True, precision=3)
        # print(rospy.Time.now().to_sec(),'\n',np.array(t.quaternion_from_matrix(robot.calculate_tf(robot.theta))))
        r.sleep()
