#!/usr/bin/python
# From https://answers.ros.org/question/27713/how-to-recover-the-saved-images-in-a-bagfile-to-jpg-or-png/

import rosbag
import rospy
import cv2
from cv_bridge import CvBridge
from pathlib import Path


def ingest_from_bag(topic, save_freq, show_progress=False):
    save_dir = Path('/home/cenkt/rnm/files/Seminar/calibration/calibration_bag_out/')
    filename = Path('/home/cenkt/rnm/files/Seminar/calibration/calibration.bag')
    bridge = CvBridge()

    message_count = 0
    with rosbag.Bag(filename, 'r') as bag:
        t0 = bag.get_start_time()
        duration = bag.get_end_time() - t0
        for topic, msg, t in bag.read_messages(topics=topic):
            if show_progress:
                progress = (t.to_time() - t0) / duration
                print(f"Progress : %{progress * 100:.0f} \r", end="")
            message_count += 1
            if message_count % save_freq == 0:
                cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
                image_name = topic.split('/')[-1] + str(message_count // save_freq).zfill(2) + ".jpg"
                cv2.imwrite(str(save_dir.joinpath(image_name)), cv_image)
            else:
                continue
            if rospy.is_shutdown():
                break
        print("Completed")


if __name__ == '__main__':
    # rospy.init_node('image_saver')
    ingest_from_bag("/k4a/rgb/image_raw", 50, True)
