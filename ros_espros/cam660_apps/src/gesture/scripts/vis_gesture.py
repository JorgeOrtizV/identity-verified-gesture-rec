#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tof_preprocessing.msg import Agent, AgentArray
from std_msgs.msg import Int32, String
from message_filters import Subscriber, ApproximateTimeSynchronizer

class GestureSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.authorized_id = None
        self.recognized_gesture = None
        self.gesture_state = None
        self.gesture_display_duration = rospy.get_param("~gesture_display_duration", 3.0)
        self.gesture_recognized_time = None

        # Sub
        self.fg_sub = Subscriber(
            "/preproc/foreground_mask",
            Image,
        )

        self.agent_sub = Subscriber(
            "/tracking/kalman",
            AgentArray,
        )

        self.auth_agent_sub = rospy.Subscriber(
            "/fusion/authorized_agent_id",
            Int32,
            self.auth_callback,
            queue_size=1
        )

        self.gesture_class_sub = rospy.Subscriber(
            "/gesture/recognized",
            String,
            self.gesture_class_callback,
            queue_size=1
        )

        self.gesture_state_sub = rospy.Subscriber(
            "/gesture/state",
            String,
            self.gesture_state_callback,
            queue_size=1
        )

        self.ts = ApproximateTimeSynchronizer(
            [self.fg_sub, self.agent_sub],
            queue_size=5,
            slop=0.1
        )

        self.ts.registerCallback(self.callback)

        rospy.loginfo("Gesture visualization node ready")

    def auth_callback(self, msg):
        self.authorized_id = msg.data

    def gesture_class_callback(self, msg):
        self.recognized_gesture = msg.data
        self.gesture_recognized_time = rospy.Time.now()

    def gesture_state_callback(self, msg):
        self.gesture_state = msg.data

    def callback(self, fg_msg, agent_msg):
        if self.authorized_id == -1:
            return

        mask = self.bridge.imgmsg_to_cv2(fg_msg, desired_encoding="passthrough")
        vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        for agent in agent_msg.agents:
            if agent.id != self.authorized_id:
                continue
            x, y, w, h = agent.x, agent.y, agent.w, agent.h
            aid = agent.id

            cv2.rectangle(vis, (x, y), (x+w, y+h), (255,0,0), 2)

            if self.gesture_recognized_time is not None:
                elapsed = (rospy.Time.now() - self.gesture_recognized_time).to_sec()
                if elapsed > self.gesture_display_duration:
                    self.recognized_gesture = None

            label_gesture = self.recognized_gesture if self.recognized_gesture else "---"
            label = f"{label_gesture} | {self.gesture_state}"
            cv2.putText(
                vis, label, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,0,0), 1
            )

        cv2.imshow("Gesture Visualization", vis)
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("vis_gesture")
    GestureSubscriber()
    rospy.spin()

