#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from simple_robot import RobotState
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np
import tf.transformations as t


if __name__ == "__main__":
    robot = RobotState()
    rospy.init_node('fk_node')
    panda_subscriber = rospy.Subscriber('/joint_states', JointState, robot.callback)
    fk_publisher = rospy.Publisher('end_effector_pose', numpy_msg(Floats), queue_size=10)
    r = rospy.Rate(1000)

    while not rospy.is_shutdown():
        fk_publisher.publish(robot.calculate_tf(robot.theta).reshape(-1))
        #np.set_printoptions(suppress=True, precision=3)
        #print(rospy.Time.now().to_sec(),'\n',np.array(t.quaternion_from_matrix(a)))
        r.sleep()
