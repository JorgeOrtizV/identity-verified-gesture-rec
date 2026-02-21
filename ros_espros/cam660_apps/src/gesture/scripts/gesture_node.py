#!/usr/bin/env python3

import rospy
import numpy as np
import os
import tf.transformations as tft
from scipy.spatial.distance import cdist

from sensor_msgs.msg import Imu
from std_msgs.msg import String, Int32
from collections import deque


class GestureNode:
    def __init__(self):
        # --- Calibration ---
        self.gravity = 9.80665
        self.imu_rate = rospy.get_param("~imu_rate", 50)  # Hz, auto-updated

        # --- Rolling buffer ---
        self.buffer_duration = rospy.get_param("~buffer_duration", 3.0)
        self.buffer_maxlen = int(self.buffer_duration * self.imu_rate)

        # --- Segmentation ---
        self.onset_thresh = rospy.get_param("~onset_thresh", 0.3) # IMU energy th to confirm onset
        self.offset_thresh = rospy.get_param("~offset_thresh", 0.15)
        self.onset_count = rospy.get_param("~onset_count", 3)      # consecutive samples to confirm onset
        self.offset_count = rospy.get_param("~offset_count", 8)    # consecutive samples to confirm offset
        self.lookback_samples = rospy.get_param("~lookback_samples", 5)
        self.min_gesture_len = rospy.get_param("~min_gesture_len", 15)   # ~0.3s at 50Hz
        self.max_gesture_len = rospy.get_param("~max_gesture_len", 200)  # ~4.0s at 50Hz
        self.energy_accel_weight = rospy.get_param("~energy_accel_weight", 0.5)
        self.max_accel = rospy.get_param("~max_accel", 9.0)
        self.max_gyro = rospy.get_param("~max_gyro", 3.5)

        # --- DTW ---
        self.template_dir = rospy.get_param("~template_dir",
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "templates"))
        self.dtw_threshold = rospy.get_param("~dtw_threshold", 0.8)
        self.strict_threshold = rospy.get_param("~strict_threshold", 0.6)
        self.dtw_band_ratio = rospy.get_param("~dtw_band_ratio", 0.5)  # Sakoe-Chiba band

        # --- Recording mode ---
        self.record_mode = rospy.get_param("~record_mode", False)
        self.record_gesture_name = rospy.get_param("~record_gesture_name", "")

        # --- Identity verification ---
        self.identity_verification = rospy.get_param("~identity_verification", False)
        self.authorized_agent_id = -1

        # --- State ---
        self.buffer = deque(maxlen=self.buffer_maxlen)
        self.state = "IDLE"
        self.onset_counter = 0
        self.offset_counter = 0
        self.gesture_samples = []

        # IMU rate auto-update
        self.last_imu_time = None
        self.imu_intervals = deque(maxlen=50)

        # Templates: {gesture_name: [np.array(N, 6), ...]}
        self.templates = {}
        if not self.record_mode:
            self.load_templates()

        # --- Subscribers ---
        self.imu_sub = rospy.Subscriber(
            "/imu/data", Imu, self.imu_callback, queue_size=100
        )

        if self.identity_verification:
            self.auth_sub = rospy.Subscriber(
                "/fusion/authorized_agent_id", Int32, self.auth_callback, queue_size=1
            )

        # --- Publishers ---
        self.gesture_pub = rospy.Publisher(
            "/gesture/recognized", String, queue_size=1
        )
        self.state_pub = rospy.Publisher(
            "/gesture/state", String, queue_size=1
        )

        mode = "RECORD" if self.record_mode else "RECOGNIZE"
        id_str = " + identity verification" if self.identity_verification else ""
        rospy.loginfo(f"Gesture node ready (mode={mode}{id_str})")
        if self.record_mode:
            rospy.loginfo(f"Recording templates for: '{self.record_gesture_name}'")
            rospy.loginfo(f"Template dir: {self.template_dir}")
        elif self.templates:
            for name, tmpls in self.templates.items():
                rospy.loginfo(f"  Loaded {len(tmpls)} templates for '{name}'")
        else:
            rospy.logwarn("No templates loaded - recognition will not work")


    # Auth callback 
    def auth_callback(self, msg):
        self.authorized_agent_id = msg.data


    # IMU callback
    def imu_callback(self, msg):
        t = msg.header.stamp.to_sec()

        # Auto-update IMU rate
        if self.last_imu_time is not None:
            interval = t - self.last_imu_time
            if 0.01 < interval < 0.1:
                self.imu_intervals.append(interval)
                if len(self.imu_intervals) >= 10:
                    self.imu_rate = 1.0 / np.mean(list(self.imu_intervals))
        self.last_imu_time = t

        # Gravity compensation via Madgwick quaternion
        q = [msg.orientation.x, msg.orientation.y,
             msg.orientation.z, msg.orientation.w]
        rot_matrix = tft.quaternion_matrix(q)[:3, :3]
        gravity_sensor = rot_matrix.T @ np.array([0, 0, self.gravity])

        ax = msg.linear_acceleration.x - gravity_sensor[0]
        ay = msg.linear_acceleration.y - gravity_sensor[1]
        az = msg.linear_acceleration.z - gravity_sensor[2]

        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        sample = np.array([ax, ay, az, wx, wy, wz])
        self.buffer.append((t, sample))

        # Normalized energy for segmentation
        accel_mag = min(np.sqrt(ax**2 + ay**2 + az**2) / self.max_accel, 1.0)
        gyro_mag = min(np.sqrt(wx**2 + wy**2 + wz**2) / self.max_gyro, 1.0)
        energy = self.energy_accel_weight * accel_mag + (1.0 - self.energy_accel_weight) * gyro_mag

        self.update_segmenter(energy, sample)


    # Segmentation state machine
    def update_segmenter(self, energy, sample):
        if self.state == "IDLE":
            if energy > self.onset_thresh:
                self.onset_counter += 1
                if self.onset_counter >= self.onset_count:
                    self.state = "ACTIVE"
                    self.onset_counter = 0
                    self.offset_counter = 0

                    # Grab lookback from rolling buffer
                    buf_list = list(self.buffer)
                    lookback_start = max(0, len(buf_list) - self.lookback_samples - self.onset_count)
                    self.gesture_samples = [s[1] for s in buf_list[lookback_start:]]

                    rospy.loginfo(f"[SEG] ONSET detected ({len(self.gesture_samples)} samples with lookback)")
                    self.state_pub.publish("ACTIVE")
            else:
                self.onset_counter = 0

        elif self.state == "ACTIVE":
            self.gesture_samples.append(sample)

            # Safety: discard if too long (not a gesture)
            if len(self.gesture_samples) >= self.max_gesture_len:
                rospy.logwarn("[SEG] Gesture too long - discarding")
                self.state = "IDLE"
                self.gesture_samples = []
                self.state_pub.publish("IDLE")
                return

            if energy < self.offset_thresh:
                self.offset_counter += 1
                if self.offset_counter >= self.offset_count:
                    self.state = "IDLE"
                    self.offset_counter = 0

                    # Trim trailing low-energy tail
                    gesture = self.gesture_samples[:-self.offset_count]
                    self.gesture_samples = []

                    self.state_pub.publish("IDLE")

                    if len(gesture) >= self.min_gesture_len:
                        rospy.loginfo(f"[SEG] OFFSET: {len(gesture)} samples captured")
                        self.process_gesture(np.array(gesture))
                    else:
                        rospy.logdebug(f"[SEG] Too short ({len(gesture)} samples) - discarding")
            else:
                self.offset_counter = 0


    # Gesture processing
    def process_gesture(self, gesture):
        gesture_norm = self.normalize(gesture)

        if self.record_mode:
            self.save_template(gesture_norm)
        else:
            if self.identity_verification and self.authorized_agent_id == -1:
                rospy.loginfo("[GESTURE] No authorized agent - discarding gesture")
                return
            self.classify(gesture_norm)

    def normalize(self, gesture):
        # Zero-mean, unit-variance per axis. Preserves temporal shape, removes amplitude differences between users.
        mean = gesture.mean(axis=0)
        std = gesture.std(axis=0)
        std[std < 1e-6] = 1.0
        return (gesture - mean) / std


    # DTW classification
    def classify(self, gesture):
        best_name = None
        best_dist = float('inf')
        best_pos = None
        results = []
        band = max(1, int(self.dtw_band_ratio * max(len(gesture), 1)))
        early_stop = False

        for name, templates in self.templates.items():
            if name == "ood":
                continue
            dists = []
            for template in templates:
                # Skip templates where length mismatch exceeds band (DTW would return inf)
                if abs(len(gesture) - len(template)) > band:
                    continue
                d = self.dtw_distance(gesture, template)
                dists.append(d)
                # Speed up: If we find a very small distance then just stop iterating and return match
                if d < self.strict_threshold:
                    early_stop = True
                    break
            if not dists:
                continue
            avg_dist = np.mean(dists)
            min_dist = np.min(dists)
            pos = np.argmin(dists)
            results.append((name, min_dist, avg_dist))

            if min_dist < best_dist:
                best_dist = min_dist
                best_name = name
                best_pos = pos

            # Global early termination: skip remaining classes if strong match found
            if early_stop:
                break

        # Log all distances for debugging
        for name, min_d, avg_d in results:
            rospy.loginfo(f"  DTW '{name}': min={min_d:.2f}, avg={avg_d:.2f}")

        if best_name is not None and best_dist < self.dtw_threshold:
            rospy.loginfo(f"RECOGNIZED: '{best_name}' (distance={best_dist:.2f}) - matching file: {(best_pos+1):03d}") # Also print the matched file for debugging purposes
            self.gesture_pub.publish(best_name)
        else:
            rospy.loginfo(f"No match (best='{best_name}', dist={best_dist:.2f}, thresh={self.dtw_threshold})")

    def dtw_distance(self, s1, s2):
        # Multivariate DTW with Sakoe-Chiba band. Distance is normalized by path length so it's comparable across gestures of different durations.
        n, m = len(s1), len(s2)
        band = max(1, int(self.dtw_band_ratio * max(n, m)))

        # Precompute all pairwise Euclidean distances in one C-level call
        D = cdist(s1, s2, 'euclidean')

        cost = np.full((n + 1, m + 1), np.inf)
        cost[0, 0] = 0.0

        for i in range(1, n + 1):
            j_start = max(1, i - band)
            j_end = min(m, i + band)
            for j in range(j_start, j_end + 1):
                cost[i, j] = D[i - 1, j - 1] + min(cost[i-1, j], cost[i, j-1], cost[i-1, j-1])

        return cost[n, m] / (n + m)


    # Template I/O
    def load_templates(self):
        if not os.path.isdir(self.template_dir):
            rospy.logwarn(f"Template directory not found: {self.template_dir}")
            return

        for gesture_name in sorted(os.listdir(self.template_dir)):
            if gesture_name == "ood":
                continue
            gesture_path = os.path.join(self.template_dir, gesture_name)
            if not os.path.isdir(gesture_path):
                continue

            templates = []
            for fname in sorted(os.listdir(gesture_path)):
                if fname.endswith(".npy"):
                    tmpl = np.load(os.path.join(gesture_path, fname))
                    templates.append(tmpl)

            if templates:
                self.templates[gesture_name] = templates

    def save_template(self, gesture):
        if not self.record_gesture_name:
            rospy.logerr("record_gesture_name not set")
            return

        save_dir = os.path.join(self.template_dir, self.record_gesture_name)
        os.makedirs(save_dir, exist_ok=True)

        existing = [f for f in os.listdir(save_dir) if f.endswith(".npy")]
        idx = len(existing) + 1
        fname = os.path.join(save_dir, f"template_{idx:03d}.npy")

        np.save(fname, gesture)
        rospy.loginfo(f"SAVED: {fname} ({gesture.shape[0]} samples, {gesture.shape[1]} axes)")


if __name__ == "__main__":
    rospy.init_node("gesture")
    GestureNode()
    rospy.spin()
