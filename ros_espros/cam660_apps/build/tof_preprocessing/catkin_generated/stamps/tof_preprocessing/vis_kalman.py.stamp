#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tof_preprocessing.msg import Agent, AgentArray
from message_filters import Subscriber, ApproximateTimeSynchronizer


class KalmanSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.fg_sub = Subscriber(
            "/preproc/foreground_mask",
            Image,
        )

        self.agent_sub = Subscriber(
            "/tracking/kalman",
            AgentArray,
        )

        self.ts = ApproximateTimeSynchronizer(
            [self.fg_sub, self.agent_sub],
            queue_size=5,
            slop=0.1
        )

        self.ts.registerCallback(self.callback)

        rospy.loginfo("Kalman visualization node ready")

    def callback(self, fg_msg, agent_msg):
        mask = self.bridge.imgmsg_to_cv2(fg_msg, desired_encoding="passthrough")
        vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        for agent in agent_msg.agents:
            x, y, w, h = agent.x, agent.y, agent.w, agent.h
            cx, cy = int(agent.cx), int(agent.cy)

            cv2.rectangle(vis, (x, y), (x+w, y+h), (0, 0, 255), 2)
            cv2.circle(vis, (cx, cy), 4, (0,0,255), -1)

            label = f"ID {agent.id} | {agent.state}"
            cv2.putText(
                vis, label, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1
            )

        cv2.imshow("Kalman Tracking", vis)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('vis_kalman')
    KalmanSubscriber()
    rospy.spin()
