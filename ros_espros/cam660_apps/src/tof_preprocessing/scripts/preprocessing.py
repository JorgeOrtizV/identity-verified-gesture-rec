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
        # Max depth to track - Found that is better to no filter out pixels based on depth for bg model, therefore a high value is selected
        self.max_depth = rospy.get_param("~max_depth", 6000)
        # Max depth for foreground pixels (filters out ground-level objects like chairs)
        self.fg_depth_thresh = rospy.get_param("~fg_depth_thresh", 1500) # mm, 1600 in multi, 1530 in single
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
        # Remove small flickering
        self.min_blob_size = rospy.get_param("~min_blob_size", 300)
        # Allow matching a track with the closest one up to a distance
        self.track_dist_thresh = rospy.get_param("~track_dist_thresh", 50)
        # Keep tracks for a time till timeout 
        self.track_timeout = rospy.get_param("~track_timeout", 15)
        # Keep age pixels in the foreground, if bigger than a threshold, update it with background
        self.fg_age_threshold = rospy.get_param("~fg_age_threshold", 50)
        self.static_scene_thresh = rospy.get_param("~static_scene_thresh", 30)
        # Split blobs
        self.min_depth_values = rospy.get_param("~min_depth_values", 100)
        self.depth_variation_thresh = rospy.get_param("~depth_variation_thresh", 300)
        self.peaks_threshold = rospy.get_param("~peaks_threshold", 0.4)
        # Single and multiagent parameters
        self.median_kernel = rospy.get_param("~median_kernel", 3)
        self.open_kernel_size = rospy.get_param("~open_kernel_size", 3)

        # Background model
        self.bg_buffer = []
        self.background = None
        self.bg_initialized = False
        self.full_update = False
        self.static_scene_counter = 0

        # Image dimensions (set from first frame)
        self.img_h = None
        self.img_w = None

        # Foreground model (initialized on first frame)
        self.fg_age = None
        
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

        # Initialize dimensions from first frame
        if self.img_h is None:
            self.img_h, self.img_w = depth.shape[:2]
            self.fg_age = np.zeros((self.img_h, self.img_w), dtype=np.uint16)
            rospy.loginfo(f"Image dimensions: {self.img_w}x{self.img_h}")

        self.frame_count+=1
        
        depth_clean = self.clean_depth(depth)
        depth_clean = self.denoise(depth_clean)

        if not self.bg_initialized:
            self.update_background_init(depth_clean)
            return

        # Detect foreground
        fg_mask = self.foreground_segmentation(depth_clean)
        # Remove foreground pixels beyond depth threshold (chairs, ground objects)
        fg_mask[(depth_clean > self.fg_depth_thresh) | (depth_clean == 0)] = 0
        # Remove small artifacts and extract blobs in single connected components pass
        fg_mask, blobs = self.extract_blobs(fg_mask)
        # Foreground aging
        self.fg_age[fg_mask>0] += 1
        self.fg_age[fg_mask==0] = 0
        # Blob update
        self.track_mask = np.zeros_like(fg_mask, dtype=bool)
        self.bridge_cleanup_mask = np.zeros((self.img_h, self.img_w), dtype=bool)
        self.update_tracks(blobs, depth_clean)

        # In case we have a super blob we have to split and clean the bridge areas
        if np.any(self.bridge_cleanup_mask):
            rospy.loginfo("Cleaning up bridge areas")
            self.force_background_update_bridge(self.bridge_cleanup_mask, depth_clean)
            fg_mask[self.bridge_cleanup_mask] = 0
            self.fg_age[self.bridge_cleanup_mask] = 0
        
        # Background update (use depth_clean to match bg model built from denoised data)
        self.update_background(depth_clean, fg_mask)

        # Publish data
        self.publish_blobs(msg.header)
        self.publish_images(depth_clean, fg_mask, msg.header)


    def clean_depth(self, depth):
        invalid = (depth < self.min_depth) | (depth > self.max_depth)
        depth[invalid] = np.nan
        return depth

    def denoise(self, depth):
        depth_filled = np.nan_to_num(depth, nan=0.0)
        return cv2.medianBlur(depth_filled, self.median_kernel) # 5 in multi, 3 in single

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

        # Opening: remove noise (3x3 to preserve thin arms ~5-6px wide)
        kernel_open = np.ones((self.open_kernel_size,self.open_kernel_size), np.uint8) # (5,5) in multiagent, (3,3) in single agent
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, kernel_open)

        # Per-component closing: fill holes within each blob without merging nearby agents
        fg = self.per_component_close(fg)

        return fg

    def per_component_close(self, fg_mask):
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(fg_mask, connectivity=8)
        result = np.zeros_like(fg_mask)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (17, 17))

        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] < self.min_blob_size:
                continue

            # Extract ROI with padding for closing to work at edges
            x = stats[i, cv2.CC_STAT_LEFT]
            y = stats[i, cv2.CC_STAT_TOP]
            w = stats[i, cv2.CC_STAT_WIDTH]
            h = stats[i, cv2.CC_STAT_HEIGHT]
            pad = 5
            x1, y1 = max(0, x - pad), max(0, y - pad)
            x2, y2 = min(self.img_w, x + w + pad), min(self.img_h, y + h + pad)

            # Close only this component within its ROI
            roi_mask = (labels[y1:y2, x1:x2] == i).astype(np.uint8) * 255
            closed = cv2.morphologyEx(roi_mask, cv2.MORPH_CLOSE, kernel)
            result[y1:y2, x1:x2] = np.maximum(result[y1:y2, x1:x2], closed)

        return result

    def extract_blobs(self, fg_mask):
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(fg_mask, connectivity=8)
        cleaned = np.zeros_like(fg_mask)
        blobs = []

        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            # Remove small flickering artifacts
            if area < self.min_blob_size:
                continue
            cleaned[labels == i] = 255

            # Only track blobs above minimum blob area
            if area < self.min_blob_area:
                continue

            x, y, w, h, _ = stats[i]
            cx, cy = centroids[i]
            blobs.append({
                "centroid": (int(cx), int(cy)),
                "bbox" : (x,y,w,h),
                "mask" : (labels==i),
                "area" : area
            })

        return cleaned, blobs

    def update_tracks(self, blobs, depth_map):
        updated_tracks = {}
        inactive_blobs = 0

        all_bridges = np.zeros((self.img_h, self.img_w), dtype=bool)
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
                # If distances are too far away, we could be merging two blobs into one
                x,y, w, h = blob["bbox"]
                aspect_ratio = max(w,h) / min(w,h) # When two agents are combined laterally distance ratio spikes
                # Debug
                #print("Area:", blob["area"])
                #print("Ratio:", aspect_ratio)
                if blob["area"] > 8000 and aspect_ratio > 1.4: #Important factor is the area
                    rospy.loginfo("TRIGGERING BLOB SPLITTING ROUTINE")
                    # Find peaks inside the blob
                    # Try depth-based split first (if depth available) - Proven to be more robust than distance transform splitting
                    split_result = None
                    if depth_map is not None:
                        split_result = self.split_blob_depth(blob, self.tracks, depth_map)
    
                    # Fall back to distance transform if depth fails
                    if split_result is None:
                        split_result = self.split_blob_dt(blob, self.tracks)
    
                    if split_result:
                        sub_blobs, bridge_mask = split_result
                        rospy.loginfo(f"Split into {len(sub_blobs)} sub-blobs")

                        all_bridges |= bridge_mask

                        for sub_blob, track_id in sub_blobs:
                            track = self.tracks[track_id]
                            track.centroid = sub_blob["centroid"]
                            track.bbox = sub_blob["bbox"]
                            track.bbox_history.append(sub_blob["bbox"])
                            track.centroid_history.append(sub_blob["centroid"])
                            track.age += 1
                            track.last_seen = self.frame_count
                            track.update_static_flag()
                            if track.is_static:
                                inactive_blobs += 1
                            updated_tracks[track_id] = track
                            self.track_mask |= (sub_blob["mask"] & ~all_bridges)
                        continue  # Don't create new track
                    else:
                        rospy.logwarn("Failed to split, creating new track")

                # Create new track (normal case or split failed)
                track = Track(self.next_track_id, blob["centroid"], blob["bbox"])
                track.bbox_history.append(blob["bbox"])
                track.centroid_history.append(blob["centroid"])
                updated_tracks[self.next_track_id] = track
                self.next_track_id += 1
            else:
                # Matching existing track found
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
            self.track_mask |= blob["mask"]

        for tid, track in self.tracks.items():
            if tid not in updated_tracks and self.frame_count-track.last_seen < self.track_timeout:
                updated_tracks[tid] = track

        if inactive_blobs == len(blobs):
            self.static_scene_counter += 1 
        else:
            self.static_scene_counter = 0
        self.full_update = self.static_scene_counter > self.static_scene_thresh

        self.tracks = updated_tracks
        self.bridge_cleanup_mask = all_bridges

    def split_blob_dt(self, blob, tracks):
        if len(tracks) == 0:
            return None

        # Find peaks using distance transform
        mask_uint8 = blob["mask"].astype(np.uint8) * 255
        dist = cv2.distanceTransform(mask_uint8, cv2.DIST_L2, 5) # Measure distance to background

        # Find local maxima (dilate and compare)
        local_max = cv2.dilate(dist, np.ones((15, 15), np.uint8))
        peaks_mask = (dist == local_max) & (dist > 10)  # Only significant peaks

        # Get peak coordinates
        peak_coords = np.column_stack(np.where(peaks_mask))

        if len(peak_coords) < 2:
            rospy.loginfo("Not enough peaks found")
            return None

        rospy.loginfo(f"Found {len(peak_coords)} peaks")

        # For each peak, find nearest existing track centroid
        peak_to_track = {}  # {peak_idx: track_id}

        for peak_idx, (py, px) in enumerate(peak_coords):
            min_dist = float('inf')
            nearest_track_id = None

            for track_id, track in tracks.items():
                tx, ty = track.centroid
                dist_to_track = np.sqrt((px - tx)**2 + (py - ty)**2)

                if dist_to_track < min_dist:
                    min_dist = dist_to_track
                    nearest_track_id = track_id

            # Only assign if close enough to an existing track
            if min_dist < self.track_dist_thresh:
                peak_to_track[peak_idx] = nearest_track_id
                rospy.loginfo(f"Peak {peak_idx} at ({px}, {py}) → Track {nearest_track_id} (dist={min_dist:.1f})")

        # Check if peaks correspond to different tracks
        if len(peak_to_track) < 2:
            rospy.loginfo("Not enough peaks matched to tracks")
            return None

        unique_tracks = set(peak_to_track.values())

        if len(unique_tracks) < 2:
            rospy.loginfo("All peaks belong to same track")
            return None

        rospy.loginfo(f"Peaks map to {len(unique_tracks)} tracks: {unique_tracks}")

        # Split blob using watershed with peaks as seeds
        markers = np.zeros((self.img_h, self.img_w), dtype=np.int32)

        for peak_idx, track_id in peak_to_track.items():
            py, px = peak_coords[peak_idx]
            # Use track_id + 1 as marker (avoid 0 which is background)
            markers[py, px] = track_id + 1

        # Watershed needs 3-channel image
        mask_3ch = cv2.cvtColor(mask_uint8, cv2.COLOR_GRAY2BGR)
        markers = cv2.watershed(mask_3ch, markers)

        # Extract sub-blobs for each track
        sub_blobs = []
        combined_sub_mask = np.zeros((self.img_h, self.img_w), dtype=bool)

        for track_id in unique_tracks:
            marker_value = track_id + 1
            sub_mask = (markers == marker_value) & blob["mask"]

            if np.sum(sub_mask) < self.min_blob_area:
                rospy.logwarn(f"Sub-blob for track {track_id} too small: {np.sum(sub_mask)} pixels")
                return None

            combined_sub_mask |= sub_mask

            # Compute properties
            coords = np.column_stack(np.where(sub_mask))
            cy, cx = coords.mean(axis=0)
            y_min, x_min = coords.min(axis=0)
            y_max, x_max = coords.max(axis=0)

            sub_blob = {
                "centroid": (int(cx), int(cy)),
                "bbox": (int(x_min), int(y_min), int(x_max - x_min), int(y_max - y_min)),
                "mask": sub_mask,
                "area": int(np.sum(sub_mask))
            }

            sub_blobs.append((sub_blob, track_id))
            rospy.loginfo(f"Sub-blob for track {track_id}: area={sub_blob['area']}, centroid={sub_blob['centroid']}")

        if len(sub_blobs) < 2:
            return None

        bridge_mask = blob["mask"] & ~combined_sub_mask
        bridge_mask |= (markers == -1) & blob["mask"]
        rospy.loginfo(f"Bridge area: {np.sum(bridge_mask)} pixels")

        return sub_blobs, bridge_mask


    def split_blob_depth(self, blob, tracks, depth_map):
        if len(tracks) == 0:
            return None

        # Extract depth values within blob
        blob_mask = blob["mask"]
        blob_depths = depth_map[blob_mask]
        blob_depths = blob_depths[np.isfinite(blob_depths)]

        if len(blob_depths) < self.min_depth_values:
            return None

        # Check depth variation (indicating multiple people)
        depth_std = np.std(blob_depths)
        depth_range = np.ptp(blob_depths)  # peak-to-peak

        if depth_range < self.depth_variation_thresh:  # Not enough depth variation
            rospy.loginfo(f"Insufficient depth variation: range={depth_range:.0f}mm")
            return None

        # Find depth peaks using inverse depth as "distance"
        # Closer objects (smaller depth) should be peaks
        # Invert depth so peaks become maxima
        depth_for_blob = depth_map.copy()
        depth_for_blob[~blob_mask] = 0

        # Invert valid depths (so closer = higher value)
        max_depth = np.nanmax(depth_for_blob[blob_mask])
        inverted_depth = np.zeros_like(depth_for_blob)
        inverted_depth[blob_mask] = max_depth - depth_for_blob[blob_mask]

        # Smooth to avoid noise peaks
        inverted_depth_smooth = cv2.GaussianBlur(inverted_depth, (11, 11), 0) # Gaussian around peaks

        # Find local maxima
        local_max = cv2.dilate(inverted_depth_smooth, np.ones((15, 15), np.uint8))
        peaks_mask = (inverted_depth_smooth == local_max) & (inverted_depth_smooth > inverted_depth_smooth.max() * self.peaks_threshold) # Threshold

        # Get peak coordinates
        peak_coords = np.column_stack(np.where(peaks_mask))

        if len(peak_coords) < 2:
            rospy.loginfo("Not enough depth peaks")
            return None

        rospy.loginfo(f"Found {len(peak_coords)} depth peaks")

        # Match peaks to existing track centroids
        peak_to_track = {}

        for peak_idx, (py, px) in enumerate(peak_coords):
            min_dist = float('inf')
            nearest_track_id = None

            for track_id, track in tracks.items():
                tx, ty = track.centroid
                dist_to_track = np.sqrt((px - tx)**2 + (py - ty)**2)

                if dist_to_track < min_dist:
                    min_dist = dist_to_track
                    nearest_track_id = track_id

            if min_dist < self.track_dist_thresh:
                peak_to_track[peak_idx] = nearest_track_id
                rospy.logdebug(f"Depth peak {peak_idx} → Track {nearest_track_id} (dist={min_dist:.1f})")

        if len(peak_to_track) < 2:
            return None

        unique_tracks = set(peak_to_track.values())

        if len(unique_tracks) < 2:
            return None

        rospy.loginfo(f"Depth peaks map to {len(unique_tracks)} tracks")

        # Use watershed on inverted depth to split
        markers = np.zeros((self.img_h, self.img_w), dtype=np.int32)

        for peak_idx, track_id in peak_to_track.items():
            py, px = peak_coords[peak_idx]
            markers[py, px] = track_id + 1

        # Convert inverted depth to uint8 for watershed
        inverted_uint8 = cv2.normalize(inverted_depth_smooth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        inverted_3ch = cv2.cvtColor(inverted_uint8, cv2.COLOR_GRAY2BGR)

        markers = cv2.watershed(inverted_3ch, markers)

        # Extract sub-blobs
        sub_blobs = []
        combined_sub_mask = np.zeros((self.img_h, self.img_w), dtype=bool)

        for track_id in unique_tracks:
            marker_value = track_id + 1
            # Only keep pixels that are in original blob AND assigned to this track
            sub_mask = (markers == marker_value) & blob_mask

            if np.sum(sub_mask) < self.min_blob_area:
                rospy.logwarn(f"Sub-blob for track {track_id} too small")
                return None

            combined_sub_mask |= sub_mask

            coords = np.column_stack(np.where(sub_mask))
            cy, cx = coords.mean(axis=0)
            y_min, x_min = coords.min(axis=0)
            y_max, x_max = coords.max(axis=0)

            sub_blob = {
                "centroid": (int(cx), int(cy)),
                "bbox": (int(x_min), int(y_min), int(x_max - x_min), int(y_max - y_min)),
                "mask": sub_mask,
                "area": int(np.sum(sub_mask))
            }

            sub_blobs.append((sub_blob, track_id))

        if len(sub_blobs) < 2:
            return None

        bridge_mask = blob_mask & ~combined_sub_mask
        bridge_mask |= (markers == -1) & blob_mask
        rospy.loginfo(f"Bridge area: {np.sum(bridge_mask)} pixels")

        return sub_blobs, bridge_mask

    def update_background(self, depth, fg_mask):
        valid = np.isfinite(depth) & (depth > 0)
        static_fg = self.fg_age > self.fg_age_threshold

        bg_mask = ((fg_mask == 0) | (static_fg & ~self.track_mask)) & valid
        if self.full_update:
            beta = self.bg_beta2
        else:
            beta = self.bg_beta
        self.background[bg_mask] = beta*self.background[bg_mask]+(1-beta)*depth[bg_mask]

    def force_background_update_bridge(self, bridge_mask, depth):
        if bridge_mask is None or not bridge_mask.any():
            return

        dilated_bridge = cv2.dilate(bridge_mask.astype(np.uint8)*255, np.ones((5,5), np.uint8))

        valid = np.isfinite(depth) & (depth > 0) & (dilated_bridge > 0)

        if not valid.any():
            return

        self.background[valid] = depth[valid]

        self.fg_age[valid] = 0

        rospy.loginfo(f"Force-updated background in {np.sum(valid)} bridge pixels")


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

