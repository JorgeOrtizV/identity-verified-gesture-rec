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
        self.imu_safety_margin = rospy.get_param("~imu_safety_margin", 0.3)
        self.vision_window_duration = rospy.get_param("~vision_window_duration", 1.0)
        self.confidence_thresh = rospy.get_param("~confidence_thresh", 0.8)
        self.tolerant_confidence_thresh = rospy.get_param("~tolerant_confidence_thresh", 0.6)
        self.reentry_confidence_thresh = rospy.get_param("~reentry_confidence_thresh", 1.2)
        self.visual_queue_size = rospy.get_param("~visual_queue_size", 10)
        self.imu_queue_size = rospy.get_param("~imu_queue_size", 200)
        self.imu_min_activity_thresh = rospy.get_param("~imu_thresh", 0.05) # Normally I get a 0.02 when totally static
        self.imu_rate = rospy.get_param("~imu_rate", 50) # Hz - updated with time
        self.max_gyro = rospy.get_param("~max_gyro", 3.5)
        self.max_accel = rospy.get_param("~max_accel", 9.0)
        self.default_max_agent_vel = rospy.get_param("~max_agent_vel", 5.0)
        self.agent_vel_alpha = rospy.get_param("~agent_vel_alpha", 0.98)
        self.default_max_fg_change = rospy.get_param("~default_max_fg_change", 100)
        self.fg_change_alpha = rospy.get_param("~fg_update_alpha", 0.99)
        self.hysteresis_delay = rospy.get_param("~hysteresis_delay", 5.0)
        self.switch_cooldown = rospy.get_param("~switch_cooldown", 3.0)
        self.temporal_weight = rospy.get_param("~temporal_weight", 0.6)
        self.gravity = 9.80665

        # Init
        self.imu_buffer = deque()
        self.agent_buffers = {}
        self.fg_buffers = {}
        self.authority_score = {}
        self.currently_authorized = None
        self.last_seen_stamp = {}
        self.current_timestamp = None
        self.prev_fg_count = {}
        self.fg_change_history = {}
        self.max_fg_change = {}
        self.max_agent_vel = {}
        # Activity history correlation (split channels) - Instead of computing correlation with max value of visual motion, always compare gyro with fg and accel with agent vel
        self.imu_accel_history = deque(maxlen=30)
        self.imu_gyro_history = deque(maxlen=30)
        self.agent_motion_history = {}
        self.agent_fg_history = {}
        # Autocompute imu rate
        self.last_imu_time = None
        self.imu_intervals = deque(maxlen=50)
        # Authorized agent lifecycle
        self.last_authorized_position = None
        self.authorized_left_scene = False
        self.time_since_exit = None
        self.authorized_since = None
        self.last_switch_time = None

        # Calibration
        self.image_width = 320
        self.fov_horizontal = 108 # WF lens
        self.height = 1.9 # meters above ground
        self.w_meters = 2*self.height*np.tan(np.radians(self.fov_horizontal/2))
        self.pixels_per_meter = self.image_width/self.w_meters
        rospy.loginfo(f"Calibration: {self.pixels_per_meter:.2f} pixels/meter at {self.height}m height")

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
            #"/imu/data_raw",
            "/imu/data",
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

        # Gravity compensation using orientation from Madgwick filter
        q = [msg.orientation.x,
             msg.orientation.y,
             msg.orientation.z,
             msg.orientation.w]

        rot_matrix = tft.quaternion_matrix(q)[:3, :3]
        gravity_sensor = rot_matrix.T @ np.array([0, 0, self.gravity])

        ax = msg.linear_acceleration.x - gravity_sensor[0]
        ay = msg.linear_acceleration.y - gravity_sensor[1]
        az = msg.linear_acceleration.z - gravity_sensor[2]

        accel = np.sqrt(ax**2 + ay**2 + az**2)

        # Angular vel
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z
        w = np.sqrt(wx**2+wy**2+wz**2)

        self.imu_buffer.append((t, accel, w))
        rospy.logdebug_throttle(2.0, f"[TIMING] IMU buffer size: {len(self.imu_buffer)}")

        self.prune_imu(t)

    def agent_callback(self, fg_msg, agent_msg):
        now = agent_msg.header.stamp.to_sec()
        self.current_timestamp = now

        rospy.loginfo_throttle(2.0, f"[TIMING] Agent callback at t={now:.3f}")

        fg_mask = self.bridge.imgmsg_to_cv2(fg_msg, desired_encoding="mono8")

        imu_window = self.get_imu_window(now-self.imu_window_duration, now)

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


        # Normalize IMU components separately for cross-modal comparison
        accel_norm = min(imu_motion["window_accel"] / self.max_accel, 1.0)
        gyro_norm = min(imu_motion["window_w"] / self.max_gyro, 1.0)

        # Balanced energy for threshold check (state-agnostic)
        imu_energy_balanced = 0.5 * accel_norm + 0.5 * gyro_norm

        debug_info = f"IMU: a={accel_norm:.2f} g={gyro_norm:.2f} | Scores: "

        # If IMU is not significant, avoid calculations.
        if imu_energy_balanced < self.imu_min_activity_thresh:
            self.select_authorized(None, imu_energy_balanced)
            return

        self.imu_accel_history.append(accel_norm)
        self.imu_gyro_history.append(gyro_norm)

        scores = {}

        for agent in agent_msg.agents:
            # Skip young agents, but not all candidates to avoid loosing walking information
            if agent.age < 5:
                rospy.logdebug(f"[FUSION] Skipping Agent {agent.id} (state={agent.state})")
                continue
            # Exploration-exploitation trade-off
            if not self.should_evaluate_agent(agent.id):
                rospy.logdebug(f"Skipping Agent {agent.id} (exploiting authorized)")
                continue

            speed = np.sqrt(agent.vx**2 + agent.vy**2) # pixel/sec
            speed_meters = speed / self.pixels_per_meter # m/s

            if agent.id not in self.agent_buffers:
                rospy.loginfo(f"[FUSION] New agent {agent.id} detected - creating buffer")
                self.agent_buffers[agent.id] = deque()
                self.fg_buffers[agent.id] = deque()
                self.authority_score[agent.id] = 0.0
                self.agent_motion_history[agent.id] = deque(maxlen=30)
                self.agent_fg_history[agent.id] = deque(maxlen=30)
                self.max_agent_vel[agent.id] = self.default_max_agent_vel

            self.last_seen_stamp[agent.id] = now
            self.agent_buffers[agent.id].append((now, speed_meters))
            self.prune_agent(agent.id, now)

            rospy.logdebug_throttle(2.0,
                f"[TIMING] Agent {agent.id} buffer size: {len(self.agent_buffers[agent.id])}"
            )

            # Get agent motion - normalize based on max observed agent speed
            agent_motion = self.compute_agent_motion(self.agent_buffers[agent.id])
            if agent_motion is None:
                continue

            # Adaptive ceiling: ensure max at least reaches previously observed value
            if agent_motion > self.max_agent_vel[agent.id]:
                self.max_agent_vel[agent.id] = max(
                    agent_motion,
                    self.agent_vel_alpha * self.max_agent_vel[agent.id] +
                    (1-self.agent_vel_alpha) * agent_motion
                )
                rospy.logdebug(f"Agent {agent.id} vel max updated: {self.max_agent_vel[agent.id]:.2f} m/s")

            agent_motion = min(agent_motion/self.max_agent_vel[agent.id], 1.0) # Normalize

            # Compute foreground energy
            fg_motion = self.agent_fg_motion(agent, fg_mask)

            # Store fg_motion in timestamped buffer for temporal correlation
            self.fg_buffers[agent.id].append((now, fg_motion))
            while self.fg_buffers[agent.id] and (now - self.fg_buffers[agent.id][0][0]) > self.vision_window_duration:
                self.fg_buffers[agent.id].popleft()

            # Add to split activity histories
            self.agent_motion_history[agent.id].append(agent_motion)
            self.agent_fg_history[agent.id].append(fg_motion)

            # --- Cross-modal energy score ---
            # accel <-> agent_motion: both capture translational movement
            # gyro  <-> fg_motion:    both capture in-place activity
            accel_match = np.exp(-1.0 * (agent_motion - accel_norm)**2)
            gyro_match = np.exp(-1.0 * (fg_motion - gyro_norm)**2)
            energy_score = 0.5 * accel_match + 0.5 * gyro_match

            # Activity state multipliers (vision-only classification)
            activity_state = self.classify_activity_state(agent_motion, fg_motion)

            if activity_state == "WALKING":
                rospy.loginfo_throttle(0.5, f"Agent {agent.id}: WALKING mode")
                if agent_motion >= 0.5 and accel_norm >= 0.5:
                    energy_score *= 1.3

            elif activity_state == "GESTURE":
                rospy.loginfo_throttle(0.5, f"Agent {agent.id}: GESTURE mode")
                if agent_motion <= 0.4 and fg_motion > 0.25 and gyro_norm > 0.5:
                    energy_score *= 1.5 # Change to 1.3 - test

            elif activity_state == "IDLE":
                rospy.loginfo_throttle(0.5, f"Agent {agent.id}: IDLE mode")
                if imu_energy_balanced < 0.1 and agent_motion < 0.1 and fg_motion < 0.3:
                    energy_score *= 1.3
                else:
                    energy_score *= 0.8

            else:
                rospy.loginfo_throttle(0.5, f"Agent {agent.id}: AMBIGUOUS mode")
                energy_score *= 0.95

            # --- Dual-channel temporal cross-correlation score ---
            temporal_score = self.compute_temporal_score(agent.id, now)

            # Long-term activity correlation bonus (split channels)
            correlation_bonus = self.compute_activity_correlation(agent.id)


            if temporal_score is not None:
                score = (1.0 - self.temporal_weight) * energy_score + self.temporal_weight * temporal_score
            else:
                score = energy_score
                correlation_bonus = min(correlation_bonus, 0.15)

            score *= (1.0 + correlation_bonus)

            scores[agent.id] = min(score, 2.0)

            rospy.loginfo_throttle(0.5,
                f"Agent {agent.id}: score={scores[agent.id]:.2f} "
                f"(energy={energy_score:.2f}, temporal={'%.2f' % temporal_score if temporal_score is not None else 'N/A'}, "
                f"corr_bonus={correlation_bonus:.2f})"
            )

            debug_info += f"[A{agent.id}: score={scores.get(agent.id, 0):.2f}, conf={self.authority_score.get(agent.id, 0):.2f}] "

        rospy.loginfo_throttle(0.5, debug_info)

        self.select_authorized(scores, imu_energy_balanced)

        if self.currently_authorized is not None:
            rospy.loginfo( # _throttle(1.0 - made info only for debugging purposes
                f"AUTHORIZED: Agent {self.currently_authorized} "
                f"(conf={self.authority_score.get(self.currently_authorized, 0):.2f})"
            )

        self.cleanup()


    def prune_imu(self, now):
        keep_duration = self.imu_window_duration + self.imu_safety_margin
        before_count = len(self.imu_buffer)
        while self.imu_buffer and (now - self.imu_buffer[0][0]) > keep_duration:
            self.imu_buffer.popleft()
        after_count = len(self.imu_buffer)

        if before_count != after_count:
            rospy.logdebug(f"[TIMING] Pruned IMU buffer: {before_count} -> {after_count} samples")


    def prune_agent(self, aid, now):
        buf = self.agent_buffers[aid]
        before_count = len(buf)
        while buf and (now-buf[0][0]) > self.vision_window_duration:
            buf.popleft()
        after_count = len(buf)
        if before_count != after_count:
            rospy.logdebug(f"[TIMING] Pruned Agent {aid} buffer: {before_count} -> {after_count} samples")


    def get_imu_window(self, t_start, t_end):
        window = [msg for msg in self.imu_buffer if t_start <= msg[0] <= t_end]
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

        if len(imu_msgs) > 1:
            time_span = imu_msgs[-1][0] - imu_msgs[0][0]
            expected_samples = time_span * self.imu_rate

            rospy.logdebug_throttle(2.0,
                f"[TIMING] IMU density: {len(imu_msgs)}/{expected_samples:.0f} samples "
                f"in {time_span:.3f}s @ {self.imu_rate:.1f}Hz"
            )

            if len(imu_msgs) < expected_samples * 0.5:
                rospy.logwarn(
                    f"IMU window sparse: {len(imu_msgs)} samples in {time_span:.2f}s "
                    f"(expected ~{expected_samples:.0f} at {self.imu_rate}Hz)"
                )

        accel = np.array([x[1] for x in imu_msgs])
        w = np.array([x[2] for x in imu_msgs])

        return {"window_accel": np.sqrt(np.mean(accel**2)), "window_w": np.sqrt(np.mean(w**2))}

    def compute_agent_motion(self, buf):
        if len(buf) < 3:
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

        # Adaptive ceiling: ensure max at least reaches observed value
        if fg_change > self.max_fg_change[agent.id]:
            self.max_fg_change[agent.id] = max(
                fg_change,
                self.fg_change_alpha * self.max_fg_change[agent.id] + (1-self.fg_change_alpha)*fg_change
            )
            rospy.loginfo(f"Agent {agent.id} FG change max updated: {self.max_fg_change[agent.id]:.1f} pixels")

        self.fg_change_history[agent.id].append(fg_change)
        self.prev_fg_count[agent.id] = current_fg_count

        fg_changes = np.array(list(self.fg_change_history[agent.id]))
        rms_change = np.sqrt(np.mean(fg_changes**2))
        norm_change = min(rms_change / self.max_fg_change[agent.id], 1.0)
        return norm_change

    def classify_activity_state(self, agent_motion, fg_motion):
        # Vision-only: avoids circular dependency with IMU weighting
        if agent_motion >= 0.6:
            return "WALKING"
        elif 0.1 < agent_motion < 0.6 and fg_motion >= 0.25:
            return "GESTURE"
        elif agent_motion < 0.1 and fg_motion < 0.4:
            return "IDLE"
        else:
            return "AMBIGUOUS"

    def compute_temporal_score(self, agent_id, now):
        """Dual-channel temporal cross-correlation:
        Channel 1: IMU accel <-> agent speed (translational)
        Channel 2: IMU gyro  <-> fg_motion  (rotational/in-place)
        Returns average of available channels."""
        t_start = now - self.imu_window_duration

        # IMU samples in window (accel and gyro)
        imu_samples = [(t, a, w) for t, a, w in self.imu_buffer if t_start <= t <= now]
        if len(imu_samples) < 5:
            return None

        imu_times = np.array([s[0] for s in imu_samples])
        imu_accel = np.array([s[1] for s in imu_samples])
        imu_gyro = np.array([s[2] for s in imu_samples])

        channel_scores = []

        # Channel 1: accel <-> agent speed
        if agent_id in self.agent_buffers:
            agent_samples = [(t, v) for t, v in self.agent_buffers[agent_id] if t >= t_start]
            if len(agent_samples) >= 3:
                agent_times = np.array([s[0] for s in agent_samples])
                agent_vals = np.array([s[1] for s in agent_samples])
                # IMU and agent sample at different frequencies.
                agent_interp = np.interp(imu_times, agent_times, agent_vals)
                corr = self._cross_correlate(imu_accel, agent_interp)
                if corr is not None:
                    channel_scores.append(corr)

        # Channel 2: gyro <-> fg_motion
        if agent_id in self.fg_buffers:
            fg_samples = [(t, v) for t, v in self.fg_buffers[agent_id] if t >= t_start]
            if len(fg_samples) >= 3:
                fg_times = np.array([s[0] for s in fg_samples])
                fg_vals = np.array([s[1] for s in fg_samples])
                fg_interp = np.interp(imu_times, fg_times, fg_vals)
                corr = self._cross_correlate(imu_gyro, fg_interp)
                if corr is not None:
                    channel_scores.append(corr)

        if not channel_scores:
            return None
        return np.mean(channel_scores)

    def _cross_correlate(self, signal_a, signal_b):
        """Normalized cross-correlation with +/-100ms lag tolerance.
        Returns best non-negative correlation (0.0 to 1.0)."""
        a_zm = signal_a - np.mean(signal_a)
        b_zm = signal_b - np.mean(signal_b)
        std_a = np.std(a_zm)
        std_b = np.std(b_zm)
        if std_a < 1e-6 or std_b < 1e-6:
            return 0.0

        max_lag = max(1, int(0.1 * self.imu_rate))
        best_corr = 0.0
        n = len(a_zm)

        for lag in range(-max_lag, max_lag + 1):
            if lag >= 0:
                seg_a = a_zm[lag:]
                seg_b = b_zm[:n - lag]
            else:
                seg_a = a_zm[:n + lag]
                seg_b = b_zm[-lag:]

            if len(seg_a) < 3:
                continue

            corr = np.dot(seg_a, seg_b) / (len(seg_a) * std_a * std_b)
            best_corr = max(best_corr, corr)

        return min(best_corr, 1.0)

    def select_authorized(self, scores, imu_energy):
        # Case 1: No scores (IMU still, no agents passed filters, or no agents visible)
        # Keep current authorization, just decay. Let cleanup() handle physical exits.
        if not scores:
            self.decay_authority(0.9995)
            if self.currently_authorized is not None:
                auth_conf = self.authority_score.get(self.currently_authorized, 0.0)
                if auth_conf >= self.tolerant_confidence_thresh:
                    self.publish_result(self.currently_authorized, auth_conf)
                else:
                    rospy.logwarn(
                        f"Agent {self.currently_authorized} confidence decayed to {auth_conf:.2f} - deauthorizing"
                    )
                    self.currently_authorized = None
                    self.authorized_since = None
                    self.publish_result(-1, auth_conf)
            else:
                self.publish_result(-1, 0.0)
            return

        # Update ALL evaluated agents' authority scores (not just best)
        for aid, s in scores.items():
            self.authority_score[aid] = 0.9 * self.authority_score.get(aid, 0.0) + 0.1 * s

        best_agent = max(scores, key=scores.get)
        best_score = scores[best_agent]

        # Case 3: Reentry - authorized agent left scene, require higher conf
        if self.authorized_left_scene and self.time_since_exit is not None:
            time_since_exit = (rospy.Time.now() - self.time_since_exit).to_sec()
            if time_since_exit < 5.0:
                rospy.loginfo(
                    f"Recent exit detected ({time_since_exit:.1f}s ago) - "
                    f"requiring {self.reentry_confidence_thresh:.1f} confidence for re-authorization"
                )

                if best_score < self.reentry_confidence_thresh:
                    self.publish_result(-1, 0.0)
                    return
                else:
                    rospy.loginfo(f"Re-authorizing Agent {best_agent} after exit")
                    self.authorized_left_scene = False
                    self.time_since_exit = None
            else:
                self.authorized_left_scene = False
                self.time_since_exit = None

        # Case 4: Hysteresis - prevent rapid switching
        if self.currently_authorized is not None and best_agent != self.currently_authorized:
            auth_duration = 0.0
            if self.authorized_since is not None:
                auth_duration = (rospy.Time.now() - self.authorized_since).to_sec()

            cooldown_active = False
            if self.last_switch_time is not None:
                time_since_switch = (rospy.Time.now() - self.last_switch_time).to_sec()
                cooldown_active = time_since_switch < self.switch_cooldown

            if auth_duration > self.hysteresis_delay or cooldown_active:
                # Compare against authorized agent's frame score if available,
                # otherwise use its authority_score as baseline
                if self.currently_authorized in scores:
                    current_agent_score = scores[self.currently_authorized]
                else:
                    current_agent_score = self.authority_score.get(self.currently_authorized, 0.0)

                if best_score < current_agent_score * 1.3:
                    rospy.logdebug(
                        f"Agent {best_agent} scored {best_score:.2f} but not 30% better "
                        f"than current Agent {self.currently_authorized} ({current_agent_score:.2f})"
                    )
                    best_agent = self.currently_authorized
                    best_score = current_agent_score
                else:
                    rospy.loginfo(
                        f"Switching: Agent {best_agent} ({best_score:.2f}) >> "
                        f"Agent {self.currently_authorized} ({current_agent_score:.2f})"
                    )

        # Case 2: Authorization decision
        confidence = self.authority_score.get(best_agent, 0.0)

        if confidence > self.confidence_thresh:
            if best_agent != self.currently_authorized:
                rospy.loginfo(f"Authorizing Agent {best_agent} (conf={confidence:.2f})")
                self.currently_authorized = best_agent
                self.authorized_since = rospy.Time.now()
                self.last_switch_time = rospy.Time.now()

                if self.authorized_left_scene:
                    self.authorized_left_scene = False
                    self.time_since_exit = None
            self.publish_result(best_agent, confidence)
        else:
            # Case 5 & 6: No agent above threshold
            if self.currently_authorized is not None:
                auth_conf = self.authority_score.get(self.currently_authorized, 0.0)
                if auth_conf >= self.tolerant_confidence_thresh:
                    # Case 5: keep trusting currently authorized
                    self.publish_result(self.currently_authorized, auth_conf)
                else:
                    # Case 6: confidence too low, deauthorize
                    rospy.logwarn(
                        f"Agent {self.currently_authorized} confidence dropped to {auth_conf:.2f} - deauthorizing"
                    )
                    self.currently_authorized = None
                    self.authorized_since = None
                    self.publish_result(-1, auth_conf)
            else:
                # Case 6: nobody authorized, nobody above threshold
                self.publish_result(-1, confidence)

    def decay_authority(self, decay):
        for aid in self.authority_score.keys():
            self.authority_score[aid] *= decay

    def compute_activity_correlation(self, agent_id):
        if len(self.imu_accel_history) < 10:
            return 0.0

        if agent_id not in self.agent_motion_history:
            return 0.0

        # Channel 1: accel <-> agent_motion
        accel_hist = list(self.imu_accel_history)
        motion_hist = list(self.agent_motion_history[agent_id])
        min_len = min(len(accel_hist), len(motion_hist))
        if min_len < 5:
            return 0.0

        corr_accel = 0.0
        a_h = accel_hist[-min_len:]
        m_h = motion_hist[-min_len:]
        if np.std(a_h) > 0.01 and np.std(m_h) > 0.01:
            corr_accel = max(0, np.corrcoef(a_h, m_h)[0, 1])

        # Channel 2: gyro <-> fg_motion
        corr_gyro = 0.0
        if agent_id in self.agent_fg_history and len(self.imu_gyro_history) >= 10:
            gyro_hist = list(self.imu_gyro_history)
            fg_hist = list(self.agent_fg_history[agent_id])
            min_len2 = min(len(gyro_hist), len(fg_hist))
            if min_len2 >= 5:
                g_h = gyro_hist[-min_len2:]
                f_h = fg_hist[-min_len2:]
                if np.std(g_h) > 0.01 and np.std(f_h) > 0.01:
                    corr_gyro = max(0, np.corrcoef(g_h, f_h)[0, 1])

        # Average of available channels
        bonus = 0.5 * (corr_accel + corr_gyro) * 0.5

        return bonus

    def should_evaluate_agent(self, agent_id):
        if self.currently_authorized is None:
            return True

        if agent_id == self.currently_authorized:
            return True

        auth_conf = self.authority_score.get(self.currently_authorized, 0.0)

        if auth_conf > 1.3:
            return np.random.random() < 0.2
        elif auth_conf > 0.8:
            return np.random.random() < 0.5
        else:
            return True


    def cleanup(self):
        if not hasattr(self, 'current_timestamp'):
            rospy.logwarn("No timestamp recorded - cleanup")
            return

        now = self.current_timestamp
        to_delete = []
        for aid, last_timestamp in self.last_seen_stamp.items():
            time_since_seen = now - last_timestamp

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
            del self.agent_buffers[aid]
            if aid in self.fg_buffers:
                del self.fg_buffers[aid]
            del self.authority_score[aid]
            del self.max_fg_change[aid]
            del self.max_agent_vel[aid]
            if aid in self.agent_motion_history:
                del self.agent_motion_history[aid]
            if aid in self.agent_fg_history:
                del self.agent_fg_history[aid]
            if aid in self.fg_change_history:
                del self.fg_change_history[aid]
            if aid in self.prev_fg_count:
                del self.prev_fg_count[aid]
            if self.currently_authorized == aid:
                rospy.logwarn(f"[FUSION] Authorized agent {aid} deleted - deauthorizing")
                self.currently_authorized = None
                self.authorized_left_scene = True
                self.authorized_since = None
                self.time_since_exit = rospy.Time.now()


    def publish_result(self, agent_id, confidence):
        self.pub_id.publish(agent_id)
        self.pub_conf.publish(confidence)


if __name__ == "__main__":
    rospy.init_node("fusion")
    FusionNode()
    rospy.spin()
