#!/usr/bin/env python3
# by cenkt

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from simple_cartographer import look_at_the_point
from simple_ik import InverseKinematics
from simple_node import Communicative_Node
from simple_robot import Robot


class Inserter(Communicative_Node):
    def __init__(self, node_name):
        super(Inserter, self).__init__(node_name)
        self.robot = Robot()
        self.target = None
        self.peak_pose = None

    def set_target(self, target):
        self.target = target

    def look_at_target(self):
        self.init_publisher()
        self.wait_for_connections()
        initial_states = rospy.wait_for_message(insertion_node.j_states_topic, JointState, rospy.Duration(10)).position
        self.robot.calculate_tf(initial_states)
        current_pose = self.robot.t_matrices[10]  # 10th is the needle
        self.peak_pose = look_at_the_point(current_pose, current_pose[:-1, -1], self.target)
        self.wait_for_connections()
        self.publish(self.peak_pose)

    def move_to_target(self, intermediate_points=100):
        line = np.linspace(self.peak_pose[:-1, -1], self.target, intermediate_points)
        insertion_plan = [self.get_joint_states()]
        for i in range(intermediate_points - 1):
            goal = self.peak_pose
            goal[:-1, -1] = line[i + 1].T
            ik = InverseKinematics(goal=goal, initial_pose=insertion_plan[-1], end_eff=10)
            next_j_states = ik.calculate_joint_parameters()
            insertion_plan.append(next_j_states)

        print("calc done")
        rospy.logwarn("IN: Calculation done")
        self.init_publisher(self.command_topic)
        self.wait_for_connections()
        final_plan = []
        for i in range(len(insertion_plan) - 1):
            final_plan.append(np.linspace(insertion_plan[i], insertion_plan[i + 1], 1000))
        final_plan = np.array(final_plan).reshape(-1, 7)
        rospy.logwarn(f"IN: Published plan with shape {final_plan.shape}")
        rospy.sleep(3)
        self.publish(final_plan)


if __name__ == "__main__":
    insertion_node = Inserter('insertion_node')
    rospy.set_param("end_effector",10)
    # at the end goal is going to be listened from a topic
    goal_point = np.array([0.5, 0, 0.3])
    # goal_point = np.array(rospy.wait_for_message("/final_goal", Float64MultiArray).data).reshape((4, 4))
    insertion_node.set_target(goal_point)
    insertion_node.look_at_target()
    insertion_node.wait_for_execution()
    insertion_node.move_to_target()
