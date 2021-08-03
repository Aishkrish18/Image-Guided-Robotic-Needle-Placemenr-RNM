#!/usr/bin/env python3
# by cenkt

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from simple_cartographer import look_at_the_point
from simple_ik import InverseKinematics
from kinematics.simple_node import Communicative_Node
from kinematics.simple_robot import Robot

if __name__ == "__main__":
    insertion_node = Communicative_Node('insertion_node')
    insertion_node.init_publisher()

    insertion_node.wait_for_connections()

    initial_states = rospy.wait_for_message(insertion_node.j_states_topic, JointState, rospy.Duration(10)).position

    # at the end goal is going to be listened from a topic
    goal_point = np.array([0.3, 0, 0])

    robot = Robot()
    robot.calculate_tf(initial_states)
    current_pose = robot.t_matrices[10]
    peak_point = current_pose[:3, 3]

    T = look_at_the_point(current_pose, peak_point, goal_point)

    insertion_node.publish(T)

    intermediate_points = 100
    line = np.linspace(T[:3, 3], goal_point, intermediate_points)

    insertion_plan = []
    insertion_plan.append(np.array(rospy.wait_for_message('/joint_states', JointState, rospy.Duration(10)).position))
    for i in range(intermediate_points-1):
        goal = T
        goal[:3, 3] = line[i+1].T

        ik = InverseKinematics(goal=goal, initial_pose=insertion_plan[-1])
        next_j_states = ik.calculate_joint_parameters()
        insertion_plan.append(next_j_states)


        #insertion_node.publish(goal)
        #rospy.loginfo(f"Intermediate point {i}")

        #msg = insertion_node.wait_for_execution()

        #if msg != "reached":
        #    rospy.logwarn("Point not reached for some reason")

    print("calc done")
    final_node = Communicative_Node()
    final_node.init_publisher(final_node.command_topic)
    final_plan = []
    for i in range(len(insertion_plan)-1):
        tmp_plan = np.linspace(insertion_plan[i],insertion_plan[i+1],100)
        final_plan.append(tmp_plan)
    final_plan = np.array(final_plan).reshape(-1,7)
    print(final_plan.shape)
    rospy.sleep(3)
    print("sending")
    final_node.publish(final_plan)

