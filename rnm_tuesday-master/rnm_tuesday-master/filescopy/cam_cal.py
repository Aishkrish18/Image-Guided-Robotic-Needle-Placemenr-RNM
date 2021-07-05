#!/usr/bin/env python
import rospy
import roslib
import cv2
import cv_bridge
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageConverter:


    def __init__(self):

        #self.image_pub = rospy.Publisher('image_topic2', Image, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/k4a/rgb/image_raw', Image, self.callback1)
        self.image_sub = rospy.Subscriber('/k4a/ir/image_raw', Image, self.callback2)

    def callback1(self, data):
        global img1
        print(data.header.seq)
        if self.ct % 50 == 0:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")  # type: img
            self.A_rgb.append(img)
        self.ct = self.ct+1
        print(self.ct)

    def callback2(self, data):
        global img2
        print(data.header.seq)
        if data.header.seq % 50 == 0:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")  # type: img
            self.A_ir.append(img)

    def print_image(self):
        print("Loop here")
        for i in self.A_rgb:
            cv2.imshow("Image_window", i)
            #cv2.imwrite("/home/rnm/Documents/filter4new1/" + str(self.images) + ".jpg", i)
            self.images = self.images+1
            cv2.waitKey(3)
        for i in self.A_ir:
            cv2.imshow("Image_window", i)
            #cv2.imwrite("/home/rnm/Documents/filter4new1/" + str(self.images) + ".jpg", i)
            self.images = self.images+1
            cv2.waitKey(3)

'''

if __name__ == '__main__':
    ic = ImageConverter()
    rospy.init_node('cam_cal', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    ic.print_image()
    cv2.destroyAllWindows()

'''