#!/usr/bin/env python3
# by cenkt

from pathlib import Path
import numpy as np
import rospy
from simple_fk import RobotState
from std_msgs.msg import Float64MultiArray, String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

if __name__ == "__main__":
    poses = []
    images = []
    rospy.init_node('hand_eye_node')

    bag_file_path = Path('/home/cenkt/rnm/files/')
    joint_poses = np.loadtxt(str(bag_file_path.joinpath('positions')))

    bridge = CvBridge()

    robot = RobotState()
    goal_publisher = rospy.Publisher('goal_pose', Float64MultiArray, queue_size=10)
    for i in range(28, len(joint_poses)):
        tf_mat = robot.calculate_tf(q=joint_poses[i])

        print('Pose', i)
        while goal_publisher.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        goal_publisher.publish(Float64MultiArray(data=np.array(tf_mat, dtype=np.float32).reshape(-1)))

        msg = rospy.wait_for_message('/execution_state', String)

        if msg == 'completed':
            images.append(bridge.imgmsg_to_cv2(rospy.wait_for_message('/k4a/rgb/image_raw', Image), "bgr8"))
            poses.append(tf_mat.flatten())

    for i in range(len(images)):
        cv2.imwrite("/home/cenkt/rnm/files/hand_eye/" + str(i) + ".jpg", images[i])

    with open(bag_file_path.joinpath('tfs.txt'), 'w') as output:
        for pose in poses:
            output.write(str(list(pose)) + '\n')

# At the end I would do the creating a sphere thing
