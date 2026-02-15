#!/usr/bin/env python3

import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image
from tof_preprocessing.msg import Blob, BlobArray, AgentMsg, AgentArray
from cv_bridge import CvBridge
from scipy.optimize import linear_sum_assignment


class Agent:
    def __init__(self, agent_id, blob, spawn_location, spawn_type):
        self.id = agent_id
        self.kf = self.init_kalman(blob.cx, blob.cy)

        # bbox estimation
        self.x = blob.x
        self.y = blob.y
        self.w = blob.w
        self.h = blob.h

        # Parameters
        self.bbox_alpha = rospy.get_param("~bbox_alpha", 0.7)
        self.hit_thresh = rospy.get_param("~hit_thresh", 10)
        self.miss_count_thresh = rospy.get_param("~miss_count_thresh", 5)

        # Agent init
        self.last_seen = rospy.Time.now()
        self.age = 1
        self.confidence = 1.0
        self.state = "CANDIDATE" # CANDIDATE, ACTIVE, LOST
        self.hits = 1
        self.miss_count = 0
        self.roi = None
        self.spawn_location = spawn_location 
        self.spawn_type = spawn_type

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

        kf.processNoiseCov = np.diag([5,5,1000,1000]).astype(np.float32) # high initial uncertainity for velocity
        kf.measurementNoiseCov = np.diag([10,10]).astype(np.float32)
        kf.errorCovPost = np.eye(4, dtype=np.float32)

        kf.statePre = np.array([[cx], [cy], [0], [0]], np.float32)
        kf.statePost = np.array([[cx], [cy], [0], [0]], np.float32)

        return kf

    def predict(self, dt):
        self.kf.transitionMatrix = np.array([
            [1,0,dt,0],
            [0,1,0,dt],
            [0,0,1,0],
            [0,0,0,1]
        ], np.float32)

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
            self.miss_count = 0
            self.state = "ACTIVE"

    def get_centroid(self):
        s = self.kf.statePost.flatten()
        return s[0], s[1]

    def mark_missed(self):
        self.miss_count+=1
        self.confidence *= 0.95
        if self.miss_count > self.miss_count_thresh:
            self.state = "LOST"


class AgentTrackerNode:
    def __init__(self):
        self.agents = {}
        self.next_id = 0

        # Image dimensions
        self.width, self.height = 320, 240
        
        # Parameters
        self.maha_thresh = rospy.get_param("~maha_thresh", 9.21)
        self.iou_thresh = rospy.get_param("~iou_thresh", 0.6)
        self.duplicate_centroid_thresh = rospy.get_param("~duplicate_centroid_thresh", 30)
        self.avg_conf_thresh = rospy.get_param("~avg_conf_thresh", 0.7)
        self.static_scene_threshold = rospy.get_param("~static_scene_threshold", 20) #px/s, being very relaxed about this because just moving the arm triggers 100+ avg scene velocity, we could increase it but distance control has proven to be really good.
        self.boundary_thresh = rospy.get_param("~boundary_thresh", 40)
        # If agent is very close to an existing agent when spawning in the center, not create
        self.proximity_thresh = rospy.get_param("~proximity_thresh", 80) # If necessary reduce it, since it is only used when spawn location is center maybe we are safe with 80.
        self.iou_center_spawn_thresh = rospy.get_param("~iou_center_spawn_thresh", 0.25)
        self.strict_close_centroids_thresh = rospy.get_param("~strict_close_centroids_thresh", 20)
        self.agent_legal_age = rospy.get_param("~agent_legal_age", 10)
        self.agent_low_conf_thresh = rospy.get_param("~agent_low_conf_thresh", 0.7)
        self.agent_cleanup_conf = rospy.get_param("~agent_cleanup_conf", 0.3)
        self.size_ratio_thresh = rospy.get_param("~size_ratio_thresh", 0.35)
        self.loose_close_centroid_thresh = rospy.get_param("~loose_close_centroid_thresh", 60)

        # ROS I/O
        # Sub
        self.sub = rospy.Subscriber(
            "/preproc/blobs",
            BlobArray,
            self.callback,
            queue_size=1
        )

        # Pub
        self.pub = rospy.Publisher(
            "/tracking/kalman",
            AgentArray,
            queue_size=1
        )


    def callback(self, msg):
        blobs = msg.blobs
        agents = list(self.agents.values())
        now = msg.header.stamp

        # Predict all agents
        for agent in agents:
            dt = (now - agent.last_seen).to_sec()
            dt = max(0.01, min(dt, 0.2))
            agent.predict(dt)

        if len(agents) == 0:
            for b in blobs:
                self.create_agent(b, agents)
            self.publish_agents(msg.header)
            return

        if len(blobs) == 0:
            for agent in agents:
                agent.mark_missed()
            self.cleanup()
            self.publish_agents(msg.header)
            return

        # Build cost matrix
        cost_matrix = self.build_cost_matrix(agents, blobs)

        # Hungarian assignment
        row, col = linear_sum_assignment(cost_matrix)

        # Group assignments per blob
        blob_to_agents = {}

        for r,c in zip(row, col):
            if cost_matrix[r,c] > self.maha_thresh:
                continue

            if not c in blob_to_agents:
                blob_to_agents[c] = []
            blob_to_agents[c].append(r)

        # Find all agents close to each blob
        for blob_idx in range(len(blobs)):
            close_agents = []
            for agent_idx in range(len(agents)):
                if cost_matrix[agent_idx, blob_idx] < self.maha_thresh:
                    close_agents.append(agent_idx)

            # If more than two agents are close to a blob we override hungarian assignment
            if len(close_agents) >= 2:
                blob_to_agents[blob_idx] = close_agents

        matched_agents = set()
        matched_blobs = set()

        for blob_idx, agent_indices in blob_to_agents.items():
            matched_blobs.add(blob_idx)
            blob = blobs[blob_idx]

            if len(agent_indices) == 1:
                # 1-to-1 match, no brainer
                agent_idx = agent_indices[0]
                agent = agents[agent_idx]
                agent.correct(blob)
                matched_agents.add(agent.id)

            elif len(agent_indices) >= 2:
                rospy.logwarn(f"Potential superblob: {len(agent_indices)} agents → blob {blob_idx}")

                avg_confidence = np.mean([agents[idx].confidence for idx in agent_indices])
                
                # Use prediction instead of corrupted observation
                if avg_confidence > self.avg_conf_thresh:
                    rospy.loginfo("Using predictions, ignoring superblob observation")
                    for agent_idx in agent_indices:
                        agent = agents[agent_idx]
                        agent.last_seen = now
                        agent.age+=1
                        agent.hits+=1

                        agent.confidence *= 0.95

                        matched_agents.add(agent.id)
                        rospy.loginfo(f"Agent {agent.id}: kept prediction, conf={agent.confidence:.2f}")
                else:
                    # Fallback: Update with same observation
                    rospy.loginfo(f"Low confidence ({avg_confidence:.2f}) → using observation")
                    for agent_idx in agent_indices:
                        agent = agents[agent_idx]
                        agent.correct(blob)
                        agent.confidence *= 0.9
                        matched_agents.add(agent.id)

        # Handle missing agents
        for agent in agents:
            if agent.id not in matched_agents:
                agent.mark_missed()

        # Create new agents for unmatchd blobs
        for i, blob in enumerate(blobs):
            if i not in matched_blobs:
                self.create_agent(blob, agents)

        # Merge duplicate agents
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


    def create_agent(self, blob, agents):
        # Check average scene motion and spawn location. 
        # If scene is mostly static and potential agent appears in the center then we don-t create
        # the agent since it would be most likely a fragment of a bigger agent due to sensor flickering
        scene_motion = self.scene_motion_level()
        is_static = scene_motion < self.static_scene_threshold
        # Debug 
        # print("Scene Motion", scene_motion)

        # Check spawn location
        spawn_location = (blob.cx, blob.cy)
        spawn_type = self.classify_spawn(spawn_location)

        # Check relative position with other agents
        blob_pos = np.array([blob.cx, blob.cy])
        proximity = False
        for agent in agents:
            agent_pos = np.array(agent.get_centroid())
            dist = np.linalg.norm(agent_pos - blob_pos)
            if dist < self.proximity_thresh:
                proximity = True


        if spawn_type == "CENTER" and is_static and len(self.agents) > 0:
            rospy.loginfo("Not creating agent: Center spawn during static scene - most likely fragment")
            return
        elif spawn_type == "CENTER" and proximity:
            rospy.loginfo("Not creating agent: Center spawn and very close to existing agent - most likely fragment")
            return

        agent = Agent(self.next_id, blob, spawn_location, spawn_type)
        self.agents[self.next_id] = agent
        self.next_id += 1
        rospy.loginfo(f"Created Agent: {agent.id}. Detected spawn location: {spawn_location} -> {spawn_type}")

    def scene_motion_level(self):
        if len(self.agents) == 0:
            return 0

        velocities = []
        for agent in self.agents.values():
            vx = agent.kf.statePost[2,0]
            vy = agent.kf.statePost[3,0]
            vel = np.sqrt(vx**2 + vy**2)
            velocities.append(vel)

        return np.mean(velocities)

    def classify_spawn(self, spawn_loc):
        near_boundary = (
            spawn_loc[0] < self.boundary_thresh or spawn_loc[0] > self.width - self.boundary_thresh or
            spawn_loc[1] < self.boundary_thresh or spawn_loc[1] > self.height - self.boundary_thresh
        )

        return "BOUNDARY" if near_boundary else "CENTER"


    def mahalanobis(self, agent, blob):
        z = np.array([[blob.cx], [blob.cy]], np.float32)

        H = agent.kf.measurementMatrix
        P = agent.kf.errorCovPre
        R = agent.kf.measurementNoiseCov

        S = H @ P @ H.T + R
        y = z - H @ agent.kf.statePre

        return np.sqrt(float(y.T @ np.linalg.inv(S) @ y))

    def merge_duplicate_agents(self):
        ids = list(self.agents.keys())
        to_remove = set()

        for i in range(len(ids)):
            if ids[i] in to_remove:
                continue

            for j in range(i+1, len(ids)):
                if ids[j] in to_remove:
                    continue

                a = self.agents.get(ids[i], None)
                b = self.agents.get(ids[j], None)

                if a is None or b is None:
                    continue

                # Use intersection over union as main reason
                iou = self.bbox_iou(a,b)
                # Use size ratio to merge small slaves with master agent
                area_a = a.w * a.h
                area_b = b.w * b.h
                size_ratio = min(area_a, area_b)/ max(area_a, area_b) if max(area_a, area_b) > 0 else 0
                # Use centroid distance as another criteria
                a_c = np.array(a.get_centroid())
                b_c = np.array(b.get_centroid())
                centroid_dist = np.linalg.norm(a_c - b_c)

                # Center spawn as extra criteria
                a_center_spawn = hasattr(a, "spawn_type") and a.spawn_type == "CENTER"
                b_center_spawn = hasattr(b, "spawn_type") and b.spawn_type == "CENTER"

                should_merge = False
                merge_reason = ""

                # Debug
                #print("IOU:", iou)
                #print("Size ratio:", size_ratio)
                #print("Centroid dist:", centroid_dist)

                # First step: Being aggressive for agents that just appear in the middle of the scene
                if (a_center_spawn or b_center_spawn) and (a.age < 10 or b.age < 10):
                    # since we are avoiding creating agents in this scenario, first check might be redundant but leaving it here just in case.
                    if centroid_dist < self.proximity_thresh:
                        should_merge = True
                        merge_reason = "suspicious_spawn_proximity"
                    elif iou > self.iou_center_spawn_thresh:
                        should_merge = True
                        merge_reason = "suspicious_spawn_overlap"
                # Conservative rules
                else:
                    # Big IoU and small centroid distance
                    if iou > self.iou_thresh and centroid_dist < self.duplicate_centroid_thresh:
                        should_merge = True
                        merge_reason = "high_iou_close_centroids"
                    # Very close centroids - Assess performance
                    elif centroid_dist < self.strict_close_centroids_thresh:
                        should_merge = True
                        merge_Reason = "very_close_centroids"
                    elif size_ratio < self.size_ratio_thresh and centroid_dist < self.loose_close_centroid_thresh:
                        should_merge = True
                        merge_reason = "fragement_detection"

                if should_merge:
                    rospy.loginfo(
                        f"Merging duplicate agents {ids[i]} & {ids[j]} "
                        f"[{merge_reason}]"
                        f"(IoU={iou:.2f}, dist={centroid_dist:.1f}px), size_ratio={size_ratio:.2f})"
                    )

                     # Prefer boundary-spawned over center-spawned
                    if hasattr(a, 'spawn_type') and hasattr(b, 'spawn_type'):
                        if a.spawn_type == 'BOUNDARY' and b.spawn_type == 'CENTER':
                            keep, remove = a, b
                        elif b.spawn_type == 'BOUNDARY' and a.spawn_type == 'CENTER':
                            keep, remove = b, a
                        else:
                            # Both same type - use age/size
                            keep, remove = (a, b) if a.age > b.age else (b, a)
                    else:
                        keep, remove = (a, b) if a.age > b.age else (b, a)
                    # Keep older/more confident agent
                    #if area_a > area_b or (area_a == area_b and a.confidence > b.confidence):
                    #    keep, remove = a, b
                    #else:
                    #    keep, remove = b, a

                    
                    keep.confidence = min(1.0, keep.confidence+remove.confidence*0.5)
                    to_remove.add(remove.id)

        for rid in to_remove:
            rospy.loginfo(f"Removed duplicate agent {rid}")
            del self.agents[rid]

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

            if (now - agent.last_seen).to_sec() > 2.0 and agent.state == "LOST":
                to_delete.append(aid)

            if agent.confidence < self.agent_cleanup_conf:
                to_delete.append(aid)

            # If we have young center agents with low confidence, remove them since they are most likely fragments
            if agent.age < self.agent_legal_age and agent.confidence < self.agent_low_conf_thresh and agent.spawn_type == "CENTER":
                rospy.loginfo(
                    f"Deleting young center-spawn agent {aid} "
                    f"(age={agent.age}, conf={agent.confidence:.2f})"
                )
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

