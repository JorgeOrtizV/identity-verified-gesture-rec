#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DepthSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber("/cam660_node/distance_image_raw", Image, self.callback, queue_size=1)

    def callback(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.process(depth)

    def process(self, depth):
        # Sanity check logging
        rospy.loginfo_throttle(1, f"Depth shape: {depth.shape}")
        rospy.loginfo_throttle(1, f"Depth type: {type(depth)}")
        rospy.loginfo_throttle(1, f"Max val: {np.max(depth)}")
        rospy.loginfo_throttle(1, f"Min val: {np.min(depth)}")
        # Display image
        depth_float = depth.astype(np.float32)
        #depth_float[depth_float == 0] = np.nan # Remove 0's
        # Normalize
        depth_norm = cv2.normalize(
            depth_float,
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
    print("Node starting...")
    rospy.init_node('depth_processing')
    print("Node initialized")
    DepthSubscriber()
    print("Subscriber created")
    rospy.spin()
