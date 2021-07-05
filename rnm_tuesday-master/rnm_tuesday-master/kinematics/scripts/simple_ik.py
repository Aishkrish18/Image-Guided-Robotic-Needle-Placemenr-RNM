#!/usr/bin/env python3

import rospy
import numpy as np
from scipy import optimize
from simple_robot import RobotState
from sensor_msgs.msg import JointState
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats


class InverseKinematics:
    def __init__(self,goal_topic):
        self.robot = RobotState()
        self.initial_pose = rospy.wait_for_message("/joint_states", JointState, rospy.Duration(10)).position
        self.goal = rospy.wait_for_message(goal_topic, numpy_msg(Floats), rospy.Duration(10)).data.reshape((4, 4))

    def calculate_joint_parameters(self):
        result = optimize.least_squares(fun=self.kinematic_cost_function, x0=self.initial_pose, method='trf',
                                        ftol=1e-6,
                                        bounds=[self.robot.joints_min, self.robot.joints_max],
                                        args=[self.goal])
        return result.x

    def kinematic_cost_function(self, theta, goal):
        current_end_effector = self.robot.calculate_tf(theta)
        diff = np.abs(current_end_effector[:-1, :] - goal[:-1, :])
        return diff.reshape(-1)


if __name__ == "__main__":
    rospy.init_node('kinematics')
    ik = InverseKinematics(goal_topic='/our_goal')
    ik_publisher = rospy.Publisher('~goal_joint_states', numpy_msg(Floats), queue_size=10)

    r = rospy.Rate(1000)
    while not rospy.is_shutdown():
        goal_joint_states = np.array(ik.calculate_joint_parameters(), dtype=np.float32)
        ik_publisher.publish(goal_joint_states)
        r.sleep()
