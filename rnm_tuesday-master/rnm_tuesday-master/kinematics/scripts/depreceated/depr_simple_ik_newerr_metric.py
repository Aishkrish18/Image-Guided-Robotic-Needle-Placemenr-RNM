#!/usr/bin/env python3
# by cenkt

import rospy
import numpy as np
from scipy import optimize
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from kinematics.simple_node import Communicative_Node
from kinematics.simple_robot import Robot


class InverseKinematics(Communicative_Node):
    def __init__(self, node_name=None, end_eff=None, **kwargs):
        super(InverseKinematics, self).__init__(node_name)
        rospy.loginfo(f"IK: Inverse kinematics started for {self.execution_target}")
        self.robot = Robot()
        self.end_eff = end_eff
        if kwargs:
            self.goal = kwargs['goal']
            self.initial_pose = kwargs['initial_pose']
            rospy.loginfo(f"IK: Predefined goal and initial pose received")
        else:
            rospy.loginfo(f"IK: Listening {self.goal_topic}")
            self.goal = np.array(rospy.wait_for_message(self.goal_topic, Float64MultiArray).data).reshape((4, 4))
            self.initial_pose = np.array(
                rospy.wait_for_message(self.j_states_topic, JointState, rospy.Duration(10)).position)
            rospy.loginfo(f"IK: Goal received from {self.goal_topic}")

    def get_initial_pose(self):
        return self.initial_pose

    def calculate_joint_parameters(self):
        result = optimize.least_squares(fun=self.kinematic_cost_function, x0=self.initial_pose, method='trf',
                                        ftol=1e-6,
                                        bounds=(self.robot.joints_min, self.robot.joints_max),
                                        args=[self.goal])
        return result.x

    def kinematic_cost_function(self, theta, goal):
        self.robot.calculate_tf(theta)
        current_end_effector = self.robot.t_matrices
        diff = np.abs(current_end_effector[8, :-1, :] - goal[:-1, :])
        return diff.reshape(-1)

    def calculate_joint_parameters_new(self):
        result = optimize.least_squares(fun=self.kinematic_cost_function_new, x0=self.initial_pose, method='trf',
                                        ftol=1e-6,
                                        bounds=(self.robot.joints_min, self.robot.joints_max),
                                        args=[self.goal])
        return result.x

    def kinematic_cost_function_new(self, theta, goal):
        self.robot.calculate_tf(theta)
        current_end_effector = self.robot.t_matrices[8]
        rotation_current = current_end_effector[:-1, :-1]
        rotation_goal = goal[:-1, :-1]
        trans_current = current_end_effector[:-1, -1]
        trans_goal = goal[:-1, -1]
        rot_error = (1 - np.diag(rotation_current @ rotation_goal)) / 2 * 10
        trans_error = np.sqrt((trans_current - trans_goal) ** 2)
        error = np.hstack([rot_error, trans_error])
        return error


if __name__ == "__main__":
    goal_pose = np.array([[0, 1., 0, 0.5],
                          [1, 0, 0., 0],
                          [0, 0., -1, 0.5],
                          [0., 0., 0., 1.]])
    goal_pose    = np.array([[0.174334, -0.06051, 0.982828, 0.4],
                     [0.972122, -0.148378, -0.181574, 0.0],
                     [0.156812, 0.987082, 0.032952, 0.6],
                     [0, 0, 0, 1]])
    init_pose = [0, 0, 0, -0.7590757423813637, 0, 0, 0]
    ik = InverseKinematics('ik_node', goal=goal_pose, initial_pose=init_pose)
    sol1 = ik.calculate_joint_parameters()
    print(sol1)
    sol2 = ik.calculate_joint_parameters_new()
    print(sol2)

    print("goal")
    print(goal_pose)
    print("old")
    ik.robot.calculate_tf(sol1)
    print(np.round(ik.robot.t_matrices[8], 4))
    print(np.sum(goal_pose - ik.robot.t_matrices[8]))
    print("neu")
    ik.robot.calculate_tf(sol2)
    print(np.round(ik.robot.t_matrices[8], 4))
    print(np.sum(goal_pose - ik.robot.t_matrices[8]))
