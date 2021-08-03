#!/usr/bin/env python3
# by cenkt

import pickle
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from simple_node import Communicative_Node
from simple_robot import Robot


class ImageCapture(Communicative_Node):
    def __init__(self, node_name):
        super(ImageCapture, self).__init__(node_name)
        self.bridge = CvBridge()
        self.data = []
        self.robot = Robot()

    def callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        return img

    def idle(self):
        msg = None
        while msg != "reached":
            msg = ic.wait_for_execution()

            if msg == 'scanning_done':
                ongoing = False

        rospy.sleep(1)

        # Real capture
        # ir_msg = rospy.wait_for_message(self.ir_topic, Image)
        rgb_msg = rospy.wait_for_message(self.rgb_topic, Image)
        # rgb_msg = rospy.wait_for_message("/stereo_driver_node/image_raw", Image)
        # ir_img = self.callback(ir_msg)
        rgb_img = self.callback(rgb_msg)
        # rgb_img = None
        j_states = np.array(rospy.wait_for_message(self.j_states_topic, JointState).position)
        self.robot.calculate_tf(j_states)
        pose = self.robot.t_matrices[8]

        # pseudo capture
        # print("**CAPTURED IMAGE**")
        # rgb_img, ir_img = None, None
        # rgb_img, ir_img = test_img, test_img
        # rospy.sleep(1)

        bundle = [rgb_img, pose]  # time
        self.data.append(bundle)
        self.execution_publisher.publish("next")
        rospy.loginfo(f"CC: Published the request for next pose")


if __name__ == '__main__':
    rospy.set_param("end_effector",9)
    ic = ImageCapture("image_capture")
    ic.init_execution_publisher()
    panda_subscriber = rospy.Subscriber('/joint_states', JointState, ic.robot.callback)

    ongoing = True
    while ongoing:
        try:
            ic.idle()
        except rospy.exceptions.ROSInterruptException:
            print("saving images")
            output_dir = "/home/cenkt/rnm/files/scanning_output/"
            with open(output_dir + "scanning.pkl", "wb") as f:
                pickle.dump(ic.data, f)
            exit("Process done")
