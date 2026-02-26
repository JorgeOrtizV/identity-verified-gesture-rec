#!/usr/bin/env python

import json
import os
from collections import Counter
import rospy
from std_msgs.msg import Int32, Float32, String
from tof_preprocessing.msg import AgentArray
from mpu.msg import ConfArray


class MetricsNode:
    def __init__(self):
        self.bag_name = rospy.get_param("~bag_name", "unnamed")
        self.start_time = None

        # Per-agent tracking
        self.agent_first_seen = {}   # agent_id -> rospy.Time
        self.agent_last_seen = {}    # agent_id -> rospy.Time

        # Authorization events: (agent_id, stamp, confidence)
        self.auth_events = []
        self.current_auth_id = -1
        self.current_auth_start = None
        self.pending_confidence = None
        self.auth_time = {}
        self.scene_conf_samples = {}  # agent_id -> [all confidence values across the scene]

        # Gesture events: (name, stamp, auth_agent_id)
        self.gesture_events = []

        # Gesture latency tracking
        self.gesture_state = "IDLE"
        self.gesture_active_start = None
        self.gesture_latencies = []

        # Sub
        rospy.Subscriber(
            "/tracking/kalman", AgentArray, self.kalman_cb, queue_size=10
        )
        rospy.Subscriber(
            "/fusion/authorized_agent_id", Int32, self.auth_id_cb, queue_size=10
        )
        rospy.Subscriber(
            "/fusion/authorized_agent_confidence", Float32, self.auth_conf_cb,
            queue_size=10
        )
        rospy.Subscriber(
            "/gesture/recognized", String, self.gesture_cb, queue_size=10
        )
        rospy.Subscriber(
            "/gesture/state", String, self.gesture_state_cb, queue_size=10
        )
        rospy.Subscriber(
            "/fusion/confidence", ConfArray, self.conf_cb, queue_size=10
        )

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo(
            "Metrics node ready (bag_name='%s')", self.bag_name
        )


    def kalman_cb(self, msg):
        # Track agent presence timing
        stamp = msg.header.stamp
        if self.start_time is None:
            self.start_time = stamp

        # Agent first seen and agent last seen
        for agent in msg.agents:
            aid = agent.id
            if aid not in self.agent_first_seen:
                self.agent_first_seen[aid] = stamp
            self.agent_last_seen[aid] = stamp

    def auth_id_cb(self, msg):
        # Record authorization transitions.
        stamp = rospy.Time.now()
        new_id = msg.data

        # Ended previous authorization
        if self.current_auth_id >= 0 and new_id != self.current_auth_id:
            # Per agent obtain the total time authorized
            self.auth_time[self.current_auth_id] = self.auth_time.get(self.current_auth_id, 0) + (stamp - self.current_auth_start).to_sec()
            self.current_auth_id = -1
            self.current_auth_start = None

        # New authorization started
        if new_id >= 0 and new_id != self.current_auth_id:
            self.current_auth_id = new_id
            self.current_auth_start = stamp
            # Confidence arrives on a separate topic; store event with
            # placeholder confidence that auth_conf_cb will fill in.
            self.auth_events.append(
                {"agent_id": new_id, "stamp": stamp, "confidence": 0.0}
            )

    def auth_conf_cb(self, msg):
        # Attach confidence to most recent auth event (only the first value)
        if self.auth_events and self.auth_events[-1]["confidence"] == 0.0:
            self.auth_events[-1]["confidence"] = msg.data

    def conf_cb(self, msg):
        # Accumulate per-agent confidence across the entire scene
        for c in msg.confs:
            self.scene_conf_samples.setdefault(c.id, []).append(c.conf)

    def gesture_cb(self, msg):
        # Record recognized gesture
        stamp = rospy.Time.now()
        self.gesture_events.append(
            {"name": msg.data, "stamp": stamp,
             "auth_agent_id": self.current_auth_id}
        )

        # Compute latency from ACTIVE start to recognition
        if self.gesture_active_start is not None:
            latency = (stamp - self.gesture_active_start).to_sec()
            self.gesture_latencies.append(latency)

    def gesture_state_cb(self, msg):
        # Track ACTIVE/IDLE transitions for latency measurement
        state = msg.data.strip().upper()
        if state == "ACTIVE" and self.gesture_state != "ACTIVE":
            self.gesture_active_start = rospy.Time.now()
        elif state == "IDLE" and self.gesture_state == "ACTIVE":
            pass  # keep active_start for latency calc until gesture_cb fires
        self.gesture_state = state


    # Report generation

    def on_shutdown(self):
        end_time = rospy.Time.now()

        # Close out any ongoing authorization window
        if self.current_auth_id >= 0 and self.current_auth_start is not None:
            self.auth_time[self.current_auth_id] = self.auth_time.get(self.current_auth_id, 0) + (end_time - self.current_auth_start).to_sec()

        if self.start_time is None:
            rospy.logwarn("Metrics node: no data received, skipping report.")
            return

        scene_duration = (end_time - self.start_time).to_sec()
        all_agent_ids = sorted(
            set(self.agent_first_seen.keys()) | set(self.agent_last_seen.keys())
        )
        authorized_agent_ids = sorted(
            set(e["agent_id"] for e in self.auth_events if e["agent_id"] >= 0)
        )
        total_auth_events = len(self.auth_events)

        total_gestures = len(self.gesture_events)
        authorized_gestures = sum(
            1 for g in self.gesture_events if g["auth_agent_id"] >= 0
        )

        # Gesture latency stats
        mean_lat = min_lat = max_lat = 0.0
        if self.gesture_latencies:
            mean_lat = sum(self.gesture_latencies) / len(self.gesture_latencies)
            min_lat = min(self.gesture_latencies)
            max_lat = max(self.gesture_latencies)

        # Per-agent stats
        per_agent = {}
        for aid in all_agent_ids:
            first = self.agent_first_seen.get(aid)
            last = self.agent_last_seen.get(aid)
            time_in_scene = (last - first).to_sec() if first and last else 0.0
            auth_time = self.auth_time.get(aid, 0)

            agent_auth_events = [
                e for e in self.auth_events if e["agent_id"] == aid
            ]
            times_authorized = len(agent_auth_events)

            agent_gestures = [
                g for g in self.gesture_events if g["auth_agent_id"] == aid
            ]

            # Average confidence at moment of authorization
            avg_auth_conf = 0.0
            if agent_auth_events:
                avg_auth_conf = (
                    sum(e["confidence"] for e in agent_auth_events)
                    / len(agent_auth_events)
                )

            # Average confidence over all scene samples for this agent
            samples = self.scene_conf_samples.get(aid, [])
            avg_scene_conf = sum(samples) / len(samples) if samples else 0.0

            per_agent[aid] = {
                "time_in_scene": time_in_scene,
                "total_time_authorized": auth_time,
                "times_authorized": times_authorized,
                "gestures": len(agent_gestures),
                "gesture_counts": dict(Counter(g["name"] for g in agent_gestures)),
                "avg_auth_confidence": avg_auth_conf,
                "avg_scene_confidence": avg_scene_conf,
            }

        # Build report
        lines = []
        lines.append("=" * 60)
        lines.append("METRICS REPORT: %s" % self.bag_name)
        lines.append("=" * 60)
        lines.append("")

        lines.append("Scene duration: %.2f s" % scene_duration)
        lines.append(
            "Agents seen: %s" % ", ".join(str(a) for a in all_agent_ids)
        )
        lines.append(
            "Authorized agents: %s"
            % ", ".join(str(a) for a in authorized_agent_ids)
        )
        lines.append("Total auth events: %d" % total_auth_events)
        lines.append(
            "Gestures: %d total, %d while authorized"
            % (total_gestures, authorized_gestures)
        )
        if self.gesture_latencies:
            lines.append(
                "Gesture latency: mean=%.3f  min=%.3f  max=%.3f s"
                % (mean_lat, min_lat, max_lat)
            )
        lines.append("")

        lines.append("--- Per-Agent ---")
        for aid in all_agent_ids:
            s = per_agent[aid]
            lines.append(
                "  Agent %d: in_scene=%.2fs, auth_count=%d, total_time_authorized=%.2fs, "
                "gestures=%d, avg_auth_conf=%.3f, avg_scene_conf=%.3f"
                % (
                    aid,
                    s["time_in_scene"],
                    s["times_authorized"],
                    s["total_time_authorized"],
                    s["gestures"],
                    s["avg_auth_confidence"],
                    s["avg_scene_confidence"],
                )
            )
        lines.append("")
        report = "\n".join(lines) + "\n"

        # Build JSON payload
        data = {
            "bag_name": self.bag_name,
            "scene_duration": scene_duration,
            "all_agent_ids": all_agent_ids,
            "authorized_agent_ids": authorized_agent_ids,
            "total_auth_events": total_auth_events,
            "total_gestures": total_gestures,
            "authorized_gestures": authorized_gestures,
            "gesture_latency": {
                "mean": mean_lat, "min": min_lat, "max": max_lat
            },
            "per_agent": {
                str(aid): per_agent[aid] for aid in all_agent_ids
            },
        }

        # Write to results directory
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        results_dir = os.path.join(pkg_dir, "results")
        if not os.path.isdir(results_dir):
            os.makedirs(results_dir)

        txt_path = os.path.join(results_dir, "%s.txt" % self.bag_name)
        with open(txt_path, "w") as f:
            f.write(report)

        json_path = os.path.join(results_dir, "%s.json" % self.bag_name)
        with open(json_path, "w") as f:
            json.dump(data, f, indent=2)

        rospy.loginfo("Metrics report written to %s and %s", txt_path, json_path)
        rospy.loginfo("\n%s", report)


if __name__ == "__main__":
    rospy.init_node("metrics_node")
    node = MetricsNode()
    rospy.spin()
