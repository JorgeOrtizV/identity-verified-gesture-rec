#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tof_preprocessing.msg import Agent, AgentArray
from mpu.msg import Conf, ConfArray
from std_msgs.msg import Int32
from message_filters import Subscriber, ApproximateTimeSynchronizer

class ConfidenceSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.authorized_id = None

        # Sub
        self.fg_sub = Subscriber(
            "/preproc/foreground_mask",
            Image,
        )

        self.agent_sub = Subscriber(
            "/tracking/kalman",
            AgentArray,
        )

        self.conf_sub = Subscriber(
            "/fusion/confidence",
            ConfArray,
        )

        self.auth_agent = rospy.Subscriber(
            "/fusion/authorized_agent_id",
            Int32,
            self.auth_callback,
            queue_size=1
        )

        self.ts = ApproximateTimeSynchronizer(
            [self.fg_sub, self.agent_sub, self.conf_sub],
            queue_size=5,
            slop=0.1
        )

        self.ts.registerCallback(self.callback)

        rospy.loginfo("Confidence visualization node ready")

    def auth_callback(self, msg):
        self.authorized_id = msg.data

    def callback(self, fg_msg, agent_msg, conf_msg):
        mask = self.bridge.imgmsg_to_cv2(fg_msg, desired_encoding="passthrough")
        vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        conf_dict = {}

        # Build conf dict
        for msg in conf_msg.confs:
            conf_dict[msg.id] = (msg.conf, msg.state)

        for agent in agent_msg.agents:
            x, y, w, h = agent.x, agent.y, agent.w, agent.h
            aid = agent.id
            
            color = (0,255,0) if aid == self.authorized_id else (0,0,255)

            cv2.rectangle(vis, (x, y), (x+w, y+h), color, 2)
            
            conf, state = conf_dict.get(aid, (0.0, "UNK"))
            label = f"ID {aid} | {conf:.2f} | {state}"
            cv2.putText(
                vis, label, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1
            )

        cv2.imshow("Confidence Visualization", vis)
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("vis_conf")
    ConfidenceSubscriber()
    rospy.spin()




