#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import tf.transformations as tft

from sensor_msgs.msg import Imu, Image
from cv_bridge import CvBridge
from collections import deque

from tof_preprocessing.msg import AgentMsg, AgentArray
from message_filters import Subscriber, ApproximateTimeSynchronizer
from std_msgs.msg import Int32, Float32, String

class FusionNode:
    def __init__(self):
        self.bridge = CvBridge()

        # Rospy params
        self.imu_window_duration = rospy.get_param("~imu_window_duration", 0.4)
        self.imu_safety_margin = rospy.get_param("~imu_safety_margin", 0.2)
        self.vision_window_duration = rospy.get_param("~vision_window_duration", 1.0)
        self.confidence_thresh = rospy.get_param("~confidence_thresh", 0.6)
        self.tolerant_confidence_thresh = rospy.get_param("~tolerant_confidence_thresh", 0.4)
        self.visual_queue_size = rospy.get_param("~visual_queue_size", 10)
        self.imu_queue_size = rospy.get_param("~imu_queue_size", 200)
        self.agent_motion_thresh = rospy.get_param("~agent_motion_thresh", 0.15)
        self.imu_lambda = rospy.get_param("~imu_lambda", 0.4)
        self.imu_thresh = rospy.get_param("~imu_thresh", 0.05) # Normally I get a 0.02 when totally static
        self.imu_rate = rospy.get_param("~imu_rate", 50) # Hz - updated with time
        self.max_imu = rospy.get_param("~max_imu", 5.0)
        self.default_max_agent_vel = rospy.get_param("~max_agent_vel", 1.2)
        self.agent_vel_alpha = rospy.get_param("~agent_vel_alpha", 0.98)
        self.default_max_fg_change = rospy.get_param("~default_max_fg_change", 100)
        self.fg_change_alpha = rospy.get_param("~fg_update_alpha", 0.99)
        self.gravity = 9.80665

        # Init
        self.imu_buffer = deque()
        self.agent_buffers = {}
        self.authority_score = {}
        self.currently_authorized = None
        self.last_seen_stamp = {}
        self.current_timestamp = None
        self.prev_fg_count = {}
        self.fg_change_history = {}
        self.max_fg_change = {}
        self.max_agent_vel = {}
        # Introducing activity history correlation
        self.imu_activity_history = deque(maxlen=30)
        self.agent_activity_history = {}
        # Autocompute imu rate
        self.last_imu_time = None
        self.imu_intervals = deque(maxlen=50)

        # Calibration
        self.image_width = 320
        self.fov_horizontal = 108 # WF lens
        self.height = 1.9 # meters above ground
        self.w_meters = 2*self.height*np.tan(np.radians(self.fov_horizontal/2))
        self.pixels_per_meter = self.image_width/self.w_meters
        rospy.loginfo(f"Calibration: {self.pixels_per_meter:.2f} pixels/meter at {self.height}m height")
        self.max_recorded_imu = 0

        # Sub
        self.fg_sub = Subscriber(
            "/preproc/foreground_mask",
            Image
        )

        self.agent_sub = Subscriber(
            "/tracking/kalman",
            AgentArray,
        )

        self.imu_sub = rospy.Subscriber(
            "/imu/data_raw",
            Imu,
            self.imu_callback,
            queue_size=self.imu_queue_size
        )

        self.ts = ApproximateTimeSynchronizer(
            [self.fg_sub, self.agent_sub],
            queue_size=self.visual_queue_size,
            slop=0.05 #50ms
        )

        self.ts.registerCallback(self.agent_callback)

        # Pub
        self.pub_id = rospy.Publisher(
            "/fusion/authorized_agent_id",
            Int32,
            queue_size=1
        )

        self.pub_conf = rospy.Publisher(
            "/fusion/authorized_agent_confidence",
            Float32,
            queue_size=1
        )

        self.pub_debug = rospy.Publisher(
            "/fusion/debug_info",
            String,
            queue_size=1
        )

        rospy.loginfo("Fusion node ready")

    def imu_callback(self, msg):
        t = msg.header.stamp.to_sec()

        # Compute IMU rate
        if self.last_imu_time is not None:
            interval = t - self.last_imu_time
            if 0.01 < interval < 0.1: # Rate between 10Hz - 100Hz
                self.imu_intervals.append(interval)
                if len(self.imu_intervals) >= 10:
                    avg_interval = np.mean(list(self.imu_intervals))
                    self.imu_rate = 1.0/avg_interval
                    rospy.logdebug_throttle(5.0, f"IMU rate: {self.imu_rate:.1f} Hz")
        
        self.last_imu_time = t

        # Transform orientation from quaternions to euler angles
#        q = [msg.orientation.x,
#             msg.orientation.y,
#             msg.orientation.z,
#             msg.orientation.w]
#
#        roll, pitch, yaw = tft.euler_from_quaternion(q)
#        degrees = np.degrees([roll, pitch, yaw])
        # Debug
        #print(degrees)

        # Linear acc
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z - self.gravity

        accel = np.sqrt(ax**2 + ay**2 + az**2)

        # Angular vel
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z
        w = np.sqrt(wx**2+wy**2+wz**2)

        #debug 
        #print("Accel:", accel)
        #print("Angular vel:", w)

        self.imu_buffer.append((t, accel, w))
        # Debug: Log buffer size
        rospy.logdebug_throttle(2.0, f"[TIMING] IMU buffer size: {len(self.imu_buffer)}")

        self.prune_imu(t)

    def agent_callback(self, fg_msg, agent_msg):
        now = agent_msg.header.stamp.to_sec()
        self.current_timestamp = now

        # Debug: Log callback timing
        rospy.loginfo_throttle(2.0, f"[TIMING] Agent callback at t={now:.3f}")

        fg_mask = self.bridge.imgmsg_to_cv2(fg_msg, desired_encoding="mono8")

        imu_window = self.get_imu_window(now-self.imu_window_duration, now)

        # Debug: Log IMU window info
        if imu_window:
            rospy.loginfo_throttle(2.0, 
                f"[TIMING] IMU window: {len(imu_window)} samples, "
                f"span=[{imu_window[0][0]:.3f}, {imu_window[-1][0]:.3f}], "
                f"duration={imu_window[-1][0] - imu_window[0][0]:.3f}s"
            )

        imu_motion = self.compute_imu_motion(imu_window)

        if imu_motion is None:
            rospy.logwarn_throttle(1.0, "[TIMING] No IMU motion computed - insufficient data")
            return

        debug_info = "IMU: "
        imu_energy = imu_motion["window_w"] #imu_motion["window_accel"] + self.imu_lambda*imu_motion["window_w"] # 
        imu_energy = min(imu_energy/self.max_imu, 1.0) # Normalize
        debug_info += f"{imu_energy:.2f} | Scores: "

        #print("IMU energy:", imu_energy)

        if imu_energy < self.imu_thresh:
            self.select_authorized(None, imu_energy)
            return
        
        imu_active_binary = 1 if imu_energy > 0.15 else 0
        self.imu_activity_history.append(imu_active_binary)

        scores = {}
        
        for agent in agent_msg.agents:
            # Skip non-ACTIVE agents
            if agent.state != "ACTIVE":
                rospy.logdebug(f"[FUSION] Skipping Agent {agent.id} (state={agent.state})")
                continue
            speed = np.sqrt(agent.vx**2 + agent.vy**2) # pixel/sec
            speed_meters = speed / self.pixels_per_meter # m/s
            #print("Speed: ", speed)
            #print("Speed in m/s: ", speed_meters)

            if agent.id not in self.agent_buffers:
                rospy.loginfo(f"[FUSION] New agent {agent.id} detected - creating buffer")
                print("New aid observed. Create new agent buffer.")
                self.agent_buffers[agent.id] = deque()
                self.authority_score[agent.id] = 0.0
                self.agent_activity_history[agent.id] = deque(maxlen=30)

            self.last_seen_stamp[agent.id] = now
            self.agent_buffers[agent.id].append((now, speed_meters))
            self.prune_agent(agent.id, now)

            # Debug: Log agent buffer size
            rospy.logdebug_throttle(2.0,
                f"[TIMING] Agent {agent.id} buffer size: {len(self.agent_buffers[agent.id])}"
            )

            agent_motion = self.compute_agent_motion(self.agent_buffers[agent.id])
            fg_motion = self.agent_fg_motion(agent, fg_mask)
        
            if agent_motion is not None:
                if agent.id not in self.max_agent_vel:
                    self.max_agent_vel[agent.id] = self.default_max_agent_vel

                if agent_motion > self.max_agent_vel[agent.id]:
                    self.max_agent_vel[agent.id] = (
                        self.agent_vel_alpha * self.max_agent_vel[agent.id] + 
                        (1-self.agent_vel_alpha) * agent_motion
                    )
                    rospy.logdebug(f"Agent {agent.id} vel max updated: {self.max_agent_vel[agent.id]:.2f} m/s")
                
                agent_motion = min(agent_motion/self.max_agent_vel[agent.id], 1.0) # Normalize
            else:
                continue

            if imu_motion is None or agent_motion is None:
                continue


            # Determine state 
            imu_active = imu_energy > 0.15  # Use raw value
            agent_active = agent_motion > 0.3  # Normalized threshold
            fg_active = fg_motion > 0.1        # Change-based threshold

            visual_activity_binary = 1 if (agent_active or fg_active) else 0
            self.agent_activity_history[agent.id].append(visual_activity_binary)

            rospy.loginfo_throttle(0.5,
                f"Agent {agent.id}: IMU={'ACT' if imu_active else 'IDLE'} "
                f"Agent={'ACT' if agent_active else 'IDLE'} "
                f"FG={'ACT' if fg_active else 'IDLE'} "
                f"(imu={imu_energy:.1f}, agent={agent_motion:.2f}, fg={fg_motion:.2f})"
            )

            # Adaptive weighting based on activity type
            if imu_active and not agent_active:
                # Gesture scenario: IMU active but agent not moving much
                w_agent, w_fg = 0.2, 0.8  # Prioritize foreground changes
                rospy.loginfo_throttle(0.5, f"Agent {agent.id}: GESTURE mode")
            elif imu_active and agent_active:
                # Walking scenario: both active
                w_agent, w_fg = 0.6, 0.4  # Prioritize agent motion
                rospy.loginfo_throttle(0.5, f"Agent {agent.id}: WALKING mode")

            else:
                # Low activity or ambiguous
                w_agent, w_fg = 0.5, 0.5
                rospy.loginfo_throttle(0.5, f"Agent {agent.id}: DEFAULT mode")

            #if agent_motion > self.agent_motion_thresh:
            #    w_agent, w_fg = 0.7, 0.3
            #else:
            #    w_agent, w_fg = 0.3, 0.7


            score = (
                w_agent * np.exp(-abs(agent_motion-imu_energy)) +
                w_fg*np.exp(-abs(fg_motion-imu_energy))
            )



            # Bonus: if any activity matches, boost score
            if (imu_active and (agent_active or fg_active)):
                score *= 1.5  # Activity alignment bonus
            elif not imu_active and not agent_active and not fg_active:
                score *= 1.2  # All static bonus
            elif imu_active and not (agent_active or fg_active):
                score *= 0.3  # Penalty: IMU moving but no visual activity

            # Correlation
            correlation_bonus = self.compute_activity_correlation(agent.id)

            score *= (1.0 + correlation_bonus)

            
            rospy.loginfo_throttle(0.5, f"Score: {score:.2f}")
            scores[agent.id] = min(score, 1.5)  # Cap at 1.0
            
            debug_info += f"[A{agent.id}: score={scores.get(agent.id, 0):.2f}, conf={self.authority_score.get(agent.id, 0):.2f}] "

        rospy.loginfo_throttle(0.5, debug_info)

        self.select_authorized(scores, imu_energy)

        if self.currently_authorized is not None:
            rospy.loginfo_throttle(1.0,
                f"✓ AUTHORIZED: Agent {self.currently_authorized} "
                f"(conf={self.authority_score.get(self.currently_authorized, 0):.2f})"
            )

        self.cleanup()
        

    def prune_imu(self, now):
        # We keep a large imu buffer and then from the buffer we only keep the messages in a time window now-window_duration. To ensure temporal consistency we relax this margin.
        keep_duration = self.imu_window_duration + self.imu_safety_margin
        before_count = len(self.imu_buffer)
        while self.imu_buffer and (now - self.imu_buffer[0][0]) > keep_duration:
            self.imu_buffer.popleft()
        after_count = len(self.imu_buffer)

        if before_count != after_count:
            rospy.logdebug(f"[TIMING] Pruned IMU buffer: {before_count} → {after_count} samples")


    def prune_agent(self, aid, now):
        buf = self.agent_buffers[aid]
        before_count = len(buf)
        while buf and (now-buf[0][0]) > self.vision_window_duration:
            buf.popleft()
        after_count = len(buf)
        if before_count != after_count:
            rospy.logdebug(f"[TIMING] Pruned Agent {aid} buffer: {before_count} → {after_count} samples")


    def get_imu_window(self, t_start, t_end):
        window = [msg for msg in self.imu_buffer if t_start <= msg[0] <= t_end]
        # Assess the window is valid
        if len(window) > 0:
            window_start = window[0][0]
            window_end = window[-1][0]
            diff = window_start - t_start
            if diff > 0.1:
                rospy.logwarn(
                   f"IMU window incomplete: requested [{t_start:.2f}, {t_end:.2f}], "
                   f"got [{window_start:.2f}, {window_end:.2f}] (missing {diff:.2f}s at start)"
                )
        return window

    def compute_imu_motion(self, imu_msgs):
        if len(imu_msgs) < 5:
            rospy.logdebug_throttle(1.0, f"[TIMING] Insufficient IMU samples: {len(imu_msgs)}")
            return None

        # Check temporal density
        if len(imu_msgs) > 1:
            time_span = imu_msgs[-1][0] - imu_msgs[0][0]
            expected_samples = time_span * self.imu_rate

            # Debug: Always log density check
            rospy.logdebug_throttle(2.0,
                f"[TIMING] IMU density: {len(imu_msgs)}/{expected_samples:.0f} samples "
                f"in {time_span:.3f}s @ {self.imu_rate:.1f}Hz"
            )

            # If we receive less than half of the expected messages we trigger a warning
            if len(imu_msgs) < expected_samples * 0.5:
                rospy.logwarn(
                    f"IMU window sparse: {len(imu_msgs)} samples in {time_span:.2f}s "
                    f"(expected ~{expected_samples:.0f} at {self.imu_rate}Hz)"
                )

        accel = np.array([x[1] for x in imu_msgs])
        w = np.array([x[2] for x in imu_msgs])

        # return RMS
        return {"window_accel": np.sqrt(np.mean(accel**2)), "window_w": np.sqrt(np.mean(w**2))}

    def compute_agent_motion(self, buf):
        if len(buf) < 3:
            #print("Agent motion: not enough buffer data")
            #print(len(buf))
            return None
        speeds = np.array([x[1] for x in buf])
        return np.sqrt(np.mean(speeds**2))

    def agent_fg_motion(self, agent, fg_mask):
        h, w = fg_mask.shape
        x1, y1 = max(0, agent.x), max(0, agent.y)
        x2, y2 = min(w, x1+agent.w), min(h, y1+agent.h)
        if x2 <= x1 or y2 <= y1 or agent.w == 0 or agent.h == 0:
            return 0.0
        
        roi = fg_mask[y1:y2, x1:x2]
        current_fg_count = np.sum(roi>0)

        if agent.id not in self.fg_change_history:
            self.fg_change_history[agent.id] = deque(maxlen=10)
            self.prev_fg_count[agent.id] = current_fg_count
            self.max_fg_change[agent.id] = self.default_max_fg_change
            return 0.0

        prev_count = self.prev_fg_count[agent.id]
        fg_change = abs(current_fg_count - prev_count)

        # Update max observed change for normalization
        if fg_change > self.max_fg_change[agent.id]:
            self.max_fg_change[agent.id] = (
                self.fg_change_alpha * self.max_fg_change[agent.id] + (1-self.fg_change_alpha)*fg_change
            )
            rospy.loginfo(f"Agent {agent.id} FG change max updated: {self.max_fg_change[agent.id]:.1f} pixels")

        self.fg_change_history[agent.id].append(fg_change)
        self.prev_fg_count[agent.id] = current_fg_count

        if len(self.fg_change_history[agent.id]) > 0:
            fg_changes = np.array(list(self.fg_change_history[agent.id]))
            rms_change = np.sqrt(np.mean(fg_changes**2))
            norm_change = min(rms_change / self.max_fg_change[agent.id], 1.0)
            return norm_change

        return 0.0

    def select_authorized(self, scores, imu_energy):
        if not scores:
            if imu_energy < self.imu_thresh:
                self.currently_authorized = None
                self.decay_authority(0.9995)
            self.publish_result(-1, 0.0)
            return

        if scores is None and self.currently_authorized is None:
            self.publish_result(-1, 0.0)
            return

        best_agent = max(scores, key=scores.get)
        self.authority_score[best_agent] = 0.95*self.authority_score.get(best_agent, 0.0) + 0.05*scores[best_agent]
        confidence = self.authority_score[best_agent]

        if imu_energy < self.imu_thresh: # Double penalty when static
            self.decay_authority(0.9998)
            confidence = self.authority_score[best_agent]

        if confidence > self.confidence_thresh:
            self.currently_authorized = best_agent
            self.publish_result(best_agent, confidence)
        else:
            if self.currently_authorized is not None and confidence >= self.tolerant_confidence_thresh:
                self.publish_result(self.currently_authorized, confidence)
                self.decay_authority(0.9995)
            elif self.currently_authorized is not None and confidence < self.tolerant_confidence_thresh:
                self.publish_result(-1, confidence)
                self.currently_authorized = None
            else:
                self.publish_result(-1, confidence)

    def decay_authority(self, decay):
        for aid in self.authority_score.keys():
            self.authority_score[aid] *= decay

    def compute_activity_correlation(self, agent_id):
        if len(self.imu_activity_history) < 10:
            return 0.0

        if agent_id not in self.agent_activity_history:
            return 0.0

        agent_hist = list(self.agent_activity_history[agent_id])
        imu_hist = list(self.imu_activity_history)

        min_len = min(len(agent_hist), len(imu_hist))
        agent_hist = agent_hist[-min_len:]
        imu_hist = imu_hist[-min_len:]

        matches = sum(a==1 for a, i in zip(agent_hist, imu_hist))
        correlation = matches / min_len

        bonus = max(0, (correlation-0.5)*1.0)

        return bonus

    def cleanup(self):
        if not hasattr(self, 'current_timestamp'):
            rospy.logwarn("No timestamp recorded - cleanup")
            return

        now = self.current_timestamp
        to_delete = []
        for aid, last_timestamp in self.last_seen_stamp.items():
            time_since_seen = now - last_timestamp

            # Debug: Log agent timing
            rospy.logdebug_throttle(2.0,
                f"[TIMING] Agent {aid}: last seen {time_since_seen:.3f}s ago"
            )

            # If agent leaves the scene reset max_fg_change and max_agent_vel
            if time_since_seen > 0.5:
                self.max_fg_change[aid] = self.default_max_fg_change
                self.max_agent_vel[aid] = self.default_max_agent_vel
                rospy.logdebug(f"[FUSION] Agent {aid} inactive >0.5s - resetting max values")

            if time_since_seen > 3.0:
                to_delete.append(aid)
                rospy.loginfo(f"[FUSION] Agent {aid} timeout (>3.0s) - deleting")


        for aid in to_delete:
            del self.last_seen_stamp[aid]
            del self.authority_score[aid]
            del self.agent_buffers[aid]
            del self.max_fg_change[aid]
            del self.max_agent_vel[aid]
            if self.currently_authorized == aid:
                rospy.logwarn(f"[FUSION] Authorized agent {aid} deleted - deauthorizing")
                self.currently_authorized = None


    def publish_result(self, agent_id, confidence):
        self.pub_id.publish(agent_id)
        self.pub_conf.publish(confidence)


if __name__ == "__main__":
    rospy.init_node("fusion")
    FusionNode()
    rospy.spin()
