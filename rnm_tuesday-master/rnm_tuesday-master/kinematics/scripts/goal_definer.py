#!/usr/bin/env python3

import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np


if __name__ == "__main__":
    goal = np.array([[0.174334, -0.06051, 0.982828, 0.264799],
                     [0.972122, -0.148378, -0.181574, 0.248867],
                     [0.156812, 0.987082, 0.032952, 0.320182],
                     [0, 0, 0, 1]])
    rospy.init_node('goal_yeller_node')
    goal_publisher = rospy.Publisher('our_goal', numpy_msg(Floats), queue_size=10)

    r = rospy.Rate(1000)
    while not rospy.is_shutdown():
        goal_publisher.publish(np.array(goal, dtype=np.float32).reshape(-1))
