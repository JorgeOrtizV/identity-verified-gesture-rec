#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DepthSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber("/preproc/depth_clean", Image, self.callback, queue_size=1)

    def callback(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.process(depth)

    def process(self, depth):
        # Display image
        depth = depth.astype(np.float32)
        depth[depth == 0] = np.nan
        # Normalize
        depth_norm = cv2.normalize(
            depth,
            None,
            alpha=0,
            beta=255,
            norm_type=cv2.NORM_MINMAX,
            dtype=cv2.CV_8U
        )

        depth_colormap = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

        cv2.imshow("Depth image", depth_colormap)
        cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('vis_depth_clean')
    DepthSubscriber()
    rospy.spin()

