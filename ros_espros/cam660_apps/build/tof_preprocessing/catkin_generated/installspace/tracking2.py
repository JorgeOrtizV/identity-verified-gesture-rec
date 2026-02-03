#!/usr/bin/env python3

import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image
from tof_preprocessing.msg import Blob, BlobArray, AgentMsg, AgentArray
from cv_bridge import CvBridge
from scipy.optimize import linear_sum_assignment


class Agent:
    def __init__(self, agent_id, blob):
        self.id = agent_id
        self.kf = self.init_kalman(blob.cx, blob.cy)

        # bbox estimation
        self.x = blob.x
        self.y = blob.y
        self.w = blob.w
        self.h = blob.h
        self.bbox_alpha = 0.7
        self.hit_thresh = 10

        self.last_seen = rospy.Time.now()
        self.age = 1
        self.confidence = 1.0
        self.state = "CANDIDATE" # CANDIDATE, ACTIVE, LOST
        self.hits = 1
        self.miss_count = 0
        self.roi = None

    def init_kalman(self, cx, cy):
        # 4 state params (x,y,vx,vy), 2 measurement params (x,y)
        kf = cv2.KalmanFilter(4, 2, 0)

        kf.transitionMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], np.float32)

        kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], np.float32)

        #kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03#0.01
        kf.processNoiseCov = np.diag([5,5,100,100]).astype(np.float32) # high initial uncertainity for velocity
        #kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.5#0.1
        kf.measurementNoiseCov = np.diag([10,10]).astype(np.float32)
        kf.errorCovPost = np.eye(4, dtype=np.float32)

        kf.statePre = np.array([[cx], [cy], [0], [0]], np.float32)
        kf.statePost = np.array([[cx], [cy], [0], [0]], np.float32)

        return kf

    def predict(self):
        self.kf.predict()
        cx, cy = self.kf.statePre[:2].flatten()
        self.x = int(cx - self.w/2)
        self.y = int(cy - self.h/2)

    def correct(self, blob):
        z = np.array([[blob.cx], [blob.cy]], np.float32)
        self.kf.correct(z)

        # Smooth bbox size
        self.w = int(self.bbox_alpha * self.w + (1 - self.bbox_alpha) * blob.w)
        self.h = int(self.bbox_alpha * self.h + (1 - self.bbox_alpha) * blob.h)

        cx, cy = self.get_centroid()
        self.x = int(cx - self.w / 2)
        self.y = int(cy - self.h / 2)

        self.last_seen = rospy.Time.now()
        self.age += 1
        self.hits += 1

        if self.hits >= self.hit_thresh:
            self.state = "ACTIVE"

    def get_centroid(self):
        s = self.kf.statePost.flatten()
        return s[0], s[1]

    def mark_missed(self):
        self.miss_count+=1
        self.confidence *= 0.95
        self.state = "LOST"


class AgentTrackerNode:
    def __init__(self):
        self.agents = {}
        self.next_id = 0

        self.maha_thresh = rospy.get_param("~maha_thresh", 9.21)
        self.iou_thresh = rospy.get_param("~iou_thresh", 0.6)
        self.gate_thresh = rospy.get_param("~gate_thresh", 60)

        self.sub = rospy.Subscriber(
            "/preproc/blobs",
            BlobArray,
            self.callback,
            queue_size=1
        )

        self.pub = rospy.Publisher(
            "/tracking/kalman",
            AgentArray,
            queue_size=1
        )
        
        # Blob-centric version : better for agent splitting
#    def callback(self, msg):
#        blobs = msg.blobs

        # Predict all agents
#        for agent in self.agents.values():
#            agent.predict()

#        associations = {}

#        for blob in blobs:
#            agent, d = self.find_best_agent(blob)

#            if agent:
#                associations.setdefault(agent.id, []).append(blob)
#            else:
#                self.create_agent(blob)

#        for aid, blob_list in associations.items():
#            agent = self.agents[aid]
#            main_blob = min(blob_list, key=lambda b: self.euclid(agent, b))
#            agent.correct(main_blob)

#        self.merge_duplicate_agents()
#        self.cleanup()
#        self.publish_agents(msg.header)

    # Agent-centric : Drawback - 1-to-1 matching
#    def callback(self, msg):
#        blobs = msg.blobs

        # Predict all agents
#        for agent in self.agents.values():
#            agent.predict()

        # Find best agent for each blob
#        assignments = {}

#        for i, blob in enumerate(blobs):
#            best_agent, best_dist = None, np.inf
#            for agent in self.agents.values():
#                d = self.mahalanobis(agent, blob)
#                if d < best_dist and d < self.maha_thresh:
#                    best_agent, best_dist = agent, d
#            if best_agent:
#                aid = best_agent.id
#                if aid not in assignments or best_dist < assignments[aid][2]:
#                    assignments[aid] = (i, blob, best_dist)

        # Correct once per agent
#        used_blobs = set()
#        for aid, (i, blob, _) in assignments.items():
#            self.agents[aid].correct(blob)
#            used_blobs.add(i)

        # Create agents only for blobs without match
#        for i, blob in enumerate(blobs):
#            if i not in used_blobs:
#                self.create_agent(blob)

#        self.merge_duplicate_agents()
#        self.cleanup()
#        self.publish_agents(msg.header)

    def callback(self, msg):
        blobs = msg.blobs
        agents = list(self.agents.values())

        # Predict
        for agent in agents:
            agent.predict()

        # Create Cost Matrix
        if len(agents) == 0:
            for b in blobs: 
                self.create_agent(b)
            self.publish_agents(msg.header)
            return

        if len(blobs) == 0:
            for agent in agents:
                agent.mark_missed()
            self.merge_duplicate_agents()
            self.cleanup()
            self.publish_agents(msg.header)
            return

        cost_matrix = self.build_cost_matrix(agents, blobs)

        # Hungarian assignment
        row, col = linear_sum_assignment(cost_matrix)
        matched_agents = set()
        matched_blobs = set()
        for r, c in zip(row, col):
            if cost_matrix[r,c] > self.gate_thresh:
                continue

            agent = agents[r]
            blob = blobs[c]

            agent.correct(blob)
            matched_agents.add(agent.id)
            matched_blobs.add(c)

        # Missed agents
        for agent in agents:
            if agent.id not in matched_agents:
                agent.mark_missed()

        # Create agents for no matches
        for i, blob in enumerate(blobs):
            if i not in matched_blobs:
                self.create_agent(blob)

        self.merge_duplicate_agents()
        self.cleanup()
        self.publish_agents(msg.header)

    def build_cost_matrix(self, agents, blobs):
        cost = np.full((len(agents), len(blobs)), 1e6, dtype=np.float32)

        for i, agent in enumerate(agents):
            for j, blob in enumerate(blobs):
                d = self.mahalanobis(agent, blob)
                if d > self.maha_thresh:
                    continue
                cost[i, j] = d

        return cost


    def create_agent(self, blob):
        agent = Agent(self.next_id, blob)
        self.agents[self.next_id] = agent
        self.next_id += 1

    def find_best_agent(self, blob):
        best = None
        min_dist = float("inf")

        for agent in self.agents.values():

            dist = self.mahalanobis(agent, blob)
            #cx, cy = agent.get_centroid()
            #dist = np.linalg.norm([blob.cx - cx, blob.cy - cy])
            print(dist)
            if dist < min_dist and dist < self.maha_thresh:
                min_dist = dist
                best = agent

        return best, min_dist

    def mahalanobis(self, agent, blob):
        z = np.array([[blob.cx], [blob.cy]], np.float32)

        H = agent.kf.measurementMatrix
        P = agent.kf.errorCovPre
        R = agent.kf.measurementNoiseCov

        S = H @ P @ H.T + R
        y = z - H @ agent.kf.statePre

        return np.sqrt(float(y.T @ np.linalg.inv(S) @ y))

    def euclid(self, agent, blob):
        cx, cy = agent.get_centroid()
        return np.linalg.norm([blob.cx - cx, blob.cy - cy])

    def merge_duplicate_agents(self):
        ids = list(self.agents.keys())
        for i in range(len(ids)):
            for j in range(i+1, len(ids)):
                a = self.agents[ids[i]]
                b = self.agents[ids[j]]

                if self.bbox_iou(a,b) > self.iou_thresh:
                    keep, remove = (a,b) if a.age > b.age else (b, a)
                    keep.confidence += remove.confidence
                    del self.agents[remove.id]

    def bbox_iou(self, a, b):
        inter_x1 = max(a.x, b.x)
        inter_y1 = max(a.y, b.y)
        inter_x2 = min(a.x+a.w, b.x+b.w)
        inter_y2 = min(a.y+a.h, b.y+b.h)

        if inter_x2 <= inter_x1 or inter_y2 <= inter_y1:
            return 0.0

        inter_area = (inter_x2-inter_x1)*(inter_y2-inter_y1)
        area_a = a.w*a.h
        area_b = b.w*b.h

        return inter_area / float(area_a + area_b - inter_area)

    def cleanup(self):
        now = rospy.Time.now()
        to_delete = []

        for aid, agent in self.agents.items():
            if (now - agent.last_seen).to_sec() > 1.0:
                agent.state = "LOST"

            if (now - agent.last_seen).to_sec() > 2.0:
                to_delete.append(aid)

        for aid in to_delete:
            del self.agents[aid]

    def publish_agents(self, header):
        msg = AgentArray()
        msg.header = header

        for agent in self.agents.values():
            a = AgentMsg()
            a.id = agent.id

            cx, cy = agent.get_centroid()
            vx = agent.kf.statePost[2, 0]
            vy = agent.kf.statePost[3, 0]

            a.cx = cx
            a.cy = cy
            a.vx = vx
            a.vy = vy

            a.x = agent.x
            a.y = agent.y
            a.w = agent.w
            a.h = agent.h

            a.age = agent.age
            a.state = agent.state

            msg.agents.append(a)

        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("tracking")
    AgentTrackerNode()
    rospy.spin()

