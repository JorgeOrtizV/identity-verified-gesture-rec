#!/usr/bin/env python3

import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from collections import deque

from tof_preprocessing.msg import Blob, BlobArray


class Track:
    def __init__(self, track_id, centroid, bbox):
        # Set params
        # Centroid memory cache, num of frames we track movement for
        self.fg_blob_mem = rospy.get_param("~fg_blob_mem", 60)
        # Bbox cache
        self.fg_bbox_mem = rospy.get_param("~fg_bbox_mem", 20)
        # Average movement between frames to consider a blob active
        self.fg_motion_threshold = rospy.get_param("~fg_motion_threshold", 1.5)
        self.fg_bbox_motion_threshold = rospy.get_param("~fg_bbox_motion_threshold", 500)

        self.id = track_id
        self.centroid = centroid
        self.bbox = bbox
        self.last_seen = 0
        self.age = 0
        self.centroid_history = deque(maxlen=self.fg_blob_mem)
        self.bbox_history = deque(maxlen=self.fg_bbox_mem)
        self.is_static = False

    def centroid_motion(self):
        if len(self.centroid_history) < 2:
            return np.inf

        diffs = [
                np.linalg.norm(
                    np.array(self.centroid_history[i])-np.array(self.centroid_history[i-1])
                    ) 
                for i in range(1, len(self.centroid_history))
                ]

        return np.mean(diffs)

    def bbox_motion(self):
        if len(self.bbox_history) < 2:
            return np.inf
        
        areas = [w*h for (_,_,w,h) in self.bbox_history]
        diffs = [abs(areas[i] - areas[i-1]) for i in range(1, len(areas))]
        return np.mean(diffs)

    def update_static_flag(self):
        if len(self.centroid_history) < self.fg_blob_mem:
            self.is_static = False
            return

        motion = self.centroid_motion()
        bbox_motion = self.bbox_motion()
        self.is_static = motion < self.fg_motion_threshold and bbox_motion < self.fg_bbox_motion_threshold



class DepthPreprocessingNode:
    def __init__(self):
        self.bridge = CvBridge()

        # set parameters
        # Min depth to track
        self.min_depth = rospy.get_param("~min_depth", 150) # mm
        # Max depth to track - Found that is better to no filter out pixels based on depth, therefore a high value is selected
        self.max_depth = rospy.get_param("~max_depth", 6000)
        # Frames used to build a background model (initialization)
        self.bg_frames = rospy.get_param("~bg_frames", 50)
        # Slow update - background_update()
        self.bg_beta = rospy.get_param("~bg_beta", 0.995)
        # Fast update - background_update()
        self.bg_beta2 = rospy.get_param("~bg_beta2", 0.9)
        # Difference threshold to consider a pixel foreground
        self.fg_threshold = rospy.get_param("~fg_threshold", 200)
        # Only consider blobs with minimum area:
        self.min_blob_area = rospy.get_param("~min_blob_area", 3000)
        # Merge blobs threshold
        self.overlap_thresh = rospy.get_param("~overlap_thresh", 0.5)
        # Allow matching a track with the closest one up to a distance
        self.track_dist_thresh = rospy.get_param("~track_dist_thresh", 50)
        # Keep tracks for a time till timeout 
        self.track_timeout = rospy.get_param("~track_timeout", 15)
        # Keep age pixels in the foreground, if bigger than a threshold, update it with background
        self.fg_age_threshold = rospy.get_param("~fg_age_threshold", 50)
        self.static_scene_thresh = rospy.get_param("~static_scene_thresh", 45)

        # Background model
        self.bg_buffer = []
        self.background = None
        self.bg_initialized = False
        self.full_update = False
        self.static_scene_counter = 0

        # Foreground model
        self.fg_age = np.zeros((240, 320), dtype=np.uint16)
        
        # Tracking model
        self.tracks = {}
        self.next_track_id = 0
        self.frame_count = 0

        # Ros IO
        self.sub = rospy.Subscriber(
            "/cam660_node/distance_image_raw",
            Image,
            self.callback,
            queue_size=1
        )

        self.pub_fg = rospy.Publisher(
            "/preproc/foreground_mask",
            Image,
            queue_size=1
        )

        self.pub_depth = rospy.Publisher(
            "/preproc/depth_clean",
            Image,
            queue_size=1
        )

        self.pub_vis = rospy.Publisher(
            "/preproc/debug_vis",
            Image,
            queue_size=1
        )

        self.pub_blobs = rospy.Publisher(
            "/preproc/blobs",
            BlobArray,
            queue_size=1
        )

        rospy.loginfo("Depth preprocessing node initialized")


    def callback(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth = depth.astype(np.float32)

        self.frame_count+=1
        
        depth_clean = self.clean_depth(depth)
        depth_clean = self.denoise(depth_clean)

        if not self.bg_initialized:
            self.update_background_init(depth_clean)
            return

        # Detect foreground
        fg_mask = self.foreground_segmentation(depth_clean)
        # Foreground aging
        self.fg_age[fg_mask>0] += 1
        self.fg_age[fg_mask==0] = 0
        # Blob detection
        blobs = self.extract_blobs(fg_mask)
        # Merge blobs that share a large overlapping percentage
        # Not the best results, I create a bunch of new tracks because centroid gets displaced when I merge... Actually overlap percentage is always small <0.3... Even though you can see a big bbox containing a small box... Maybe then just check centroids and if other centroids are very close to the main box (locked in by tracker, then consider the mask inside that bbox as well for classification)
        blobs = self.merge_overlapping_blobs(blobs)
        # Blob update
        self.track_mask = np.zeros_like(fg_mask, dtype=bool)
        self.update_tracks(blobs)
        self.publish_blobs(msg.header)
        # Background update
        self.update_background(depth, fg_mask)


        self.publish_images(depth_clean, fg_mask, msg.header)


    def clean_depth(self, depth):
        invalid = (depth < self.min_depth) | (depth > self.max_depth)
        depth[invalid] = np.nan
        return depth

    def denoise(self, depth):
        depth_filled = np.nan_to_num(depth, nan=0.0)
        return cv2.medianBlur(depth_filled, 5) # Try with 3 kernel size

    def update_background_init(self, depth):
        self.bg_buffer.append(depth)
        
        if len(self.bg_buffer) >= self.bg_frames:
            self.background = np.nanmedian(np.stack(self.bg_buffer), axis=0)
            self.bg_initialized = True
            self.bg_buffer = []
            rospy.loginfo("Background model initialized")

    def foreground_segmentation(self, depth):
        diff = np.abs(depth - self.background)
        diff = np.nan_to_num(diff, nan=0.0)
        fg = diff > self.fg_threshold

        fg = fg.astype(np.uint8)*255

        kernel = np.ones((5,5), np.uint8)
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, kernel)
        fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, kernel)
        # Add a second closing to avoid having blobs in the middle of a white area
        fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, np.ones((9,9), np.uint8))# before was (9,9) let's see how it goes with this kernel

        return fg

    def extract_blobs(self, fg_mask):
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(fg_mask)
        blobs = []

        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            if area < self.min_blob_area:
                continue

            x, y, w, h, _ = stats[i]
            cx, cy = centroids[i]
            blobs.append({
                "centroid": (int(cx), int(cy)),
                "bbox" : (x,y,w,h),
                "mask" : (labels==i)
            })

        return blobs

    def bbox_overlap(self, a, b):
        ax, ay, aw, ah = a
        bx, by, bw, bh = b

        inter_x1 = max(ax, bx)
        inter_y1 = max(ay, by)
        inter_x2 = min(ax+aw, bx+bw)
        inter_y2 = min(ay+ah, by+bh)

        if inter_x2 <= inter_x1 or inter_y2 <= inter_y1:
            return 0.0

        inter_area = (inter_x2-inter_x1)*(inter_y2-inter_y1)
        area_a = aw*ah
        area_b = bw*bh

        return inter_area/min(area_a, area_b)

    def merge_overlapping_blobs(self, blobs):
        merged = []
        used = set()

        for i, b1 in enumerate(blobs):
            if i in used:
                continue

            x1, y1, w1, h1 = b1["bbox"]
            mask = b1["mask"].copy()
            area = np.sum(mask)

            for j, b2 in enumerate(blobs):
                if j <= i or j in used:
                    continue

                # print(self.bbox_overlap(b1["bbox"], b2["bbox"]))

                if self.bbox_overlap(b1["bbox"], b2["bbox"]) > self.overlap_thresh:
                    # print("merging blobs")
                    used.add(j)
                    
                    x2, y2, w2, h2 = b2["bbox"]
                    x_min = min(x1, x2)
                    y_min = min(y1, y2)
                    x_max = max(x1+w1, x2+w2)
                    y_max = max(y1+h1, y2+h2)
                    x1, y1 = x_min, y_min
                    w1, h1 = x_max-x_min, y_max-y_min

                    mask |= b2["mask"]
                    area += np.sum(b2["mask"])

            cx, cy = np.mean(np.column_stack(np.where(mask)), axis=0)[::-1]

            merged.append({
                "centroid": (int(cx), int(cy)),
                "bbox": (x1, y1, w1, h1),
                "mask" : mask
            })

        return merged


    def update_tracks(self, blobs):
        updated_tracks = {}
        inactive_blobs = 0
        for blob in blobs:
            matched_id = None
            min_dist = float("inf")
            # Calculate the distance of each blob with each previously registered blob, match blob with min dist blob
            for tid, track in self.tracks.items():
                dist = np.linalg.norm(np.array(blob["centroid"]) - np.array(track.centroid))
                if dist < min_dist and dist < self.track_dist_thresh:
                    min_dist = dist
                    matched_id = tid

            if matched_id is None:
                track = Track(self.next_track_id, blob["centroid"], blob["bbox"])
                track.bbox_history.append(blob["bbox"])
                track.centroid_history.append(blob["centroid"])
                updated_tracks[self.next_track_id] = track
                self.next_track_id += 1
            else:
                track = self.tracks[matched_id]
                track.centroid = blob["centroid"]
                track.bbox = blob["bbox"]
                track.bbox_history.append(blob["bbox"])
                track.centroid_history.append(blob["centroid"])
                track.age+=1
                track.last_seen = self.frame_count
                track.update_static_flag()
                if track.is_static:
                    inactive_blobs+=1
                updated_tracks[matched_id] = track
            # Don't update zones where there-s a blob
            x,y,w,h = track.bbox
            self.track_mask[y:y+h, x:x+w] = True

        for tid, track in self.tracks.items():
            if tid not in updated_tracks and self.frame_count-track.last_seen < self.track_timeout:
                updated_tracks[tid] = track

        if inactive_blobs == len(blobs):
            self.static_scene_counter += 1 
        else:
            self.static_scene_counter = 0
        self.full_update = self.static_scene_counter > self.static_scene_thresh
        #if inactive_blobs == len(blobs) and len(blobs)>0:
         #   self.full_update=True
        #else:
         #   self.full_update=False

        self.tracks = updated_tracks

    def update_background(self, depth, fg_mask):
        valid = np.isfinite(depth) & (depth > 0)
        static_fg = self.fg_age > self.fg_age_threshold
        bg_mask = ((fg_mask == 0) | (static_fg & ~self.track_mask)) & valid
        if self.full_update:
            #print("Static scene")
        #    bg_mask = ((fg_mask == 0) | static_fg) & valid
            beta = self.bg_beta2
        else:
            #print("Active scene")
        #    bg_mask = (fg_mask == 0) & (~self.track_mask) & valid
            beta = self.bg_beta
        # bg_mask = (self.fg_persistent==0) & valid
        self.background[bg_mask] = beta*self.background[bg_mask]+(1-beta)*depth[bg_mask]


    def publish_images(self, depth, fg_mask, header):
        depth_msg = self.bridge.cv2_to_imgmsg(
            np.nan_to_num(depth, nan=0).astype(np.uint16),
            encoding="mono16"
        )

        depth_msg.header = header
        self.pub_depth.publish(depth_msg)

        fg_msg = self.bridge.cv2_to_imgmsg(fg_mask, encoding="mono8")
        fg_msg.header = header
        self.pub_fg.publish(fg_msg)

        # Bounding boxes
        vis = cv2.cvtColor((fg_mask>0).astype(np.uint8)*255, cv2.COLOR_GRAY2BGR)
        for track in self.tracks.values():
            x, y, w, h = track.bbox
            motion = not track.is_static
            cx, cy = track.centroid
            cv2.rectangle(vis, (x,y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(vis, f"ID {track.id}", (x,y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
            cv2.putText(vis, f"Active blob: {motion}", (x, y+h+10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
            cv2.circle(vis, (cx, cy), 5, (0,255,0),-1)
        vis_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        vis_msg.header=header
        self.pub_vis.publish(vis_msg)


    def publish_blobs(self, header):
        msg = BlobArray()
        msg.header = header

        for track in self.tracks.values():
            b = Blob()
            b.id = track.id
            b.cx, b.cy = track.centroid
            x, y, w, h = track.bbox
            b.x, b.y, b.w, b.h = x, y, w, h
            b.area = w*h
            b.is_static = track.is_static

            msg.blobs.append(b)

        self.pub_blobs.publish(msg)


if __name__ == "__main__":
    rospy.init_node("preprocessing")
    DepthPreprocessingNode()
    rospy.spin()

