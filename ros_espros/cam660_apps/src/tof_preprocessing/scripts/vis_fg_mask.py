#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DepthSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber("/preproc/foreground_mask", Image, self.callback, queue_size=1)
        cv2.namedWindow("Mask image", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Mask image", 640, 480)
        cv2.moveWindow("Mask image", 660, 0)

    def callback(self, msg):
        mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.process(mask)

    def process(self, mask):
        cv2.imshow("Mask image", mask)
        cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('vis_fg_mask')
    DepthSubscriber()
    rospy.spin()

