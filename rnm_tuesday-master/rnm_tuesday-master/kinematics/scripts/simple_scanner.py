#!/usr/bin/env python3
# by cenkt

import numpy as np
import rosparam
import rospy
from sensor_msgs.msg import JointState
from simple_cartographer import spherical_to_cartesian, look_at_the_point
from simple_node import Communicative_Node
from simple_robot import Robot

if __name__ == "__main__":
    node = Communicative_Node("spherical_scan_node")
    node.init_publisher(node.goal_topic)
    node.init_execution_publisher()

    node.wait_for_connections()

    initial_states = rospy.wait_for_message(node.j_states_topic, JointState, rospy.Duration(10)).position
    robot = Robot()
    robot.calculate_tf(initial_states)
    last_pose = robot.t_matrices[9]

    target = np.array([0.30, 0, 0.00125])
    i = 0
    for phi in np.linspace(-0.9*np.pi / 2, 0.9*np.pi / 2, 4):
        for theta in np.linspace(np.pi / 8, np.pi / 2.75, 4):
            for r in np.linspace(0.4, 0.6, 3):
                rospy.loginfo(f"SC: Heading to pose {i}")
                peak_point = spherical_to_cartesian(target, r, theta, phi)
                T = look_at_the_point(last_pose, peak_point, target)

                node.publish(T)

                msg = None
                while msg != "next":
                    rospy.loginfo(f"SC: Waiting for execution")
                    msg = node.wait_for_execution()

                rospy.sleep(1)
                last_pose = T
                i = i + 1

    ## How to publish a single message? without initiating a publisher
    node.execution_publisher.publish("scanning_done")
