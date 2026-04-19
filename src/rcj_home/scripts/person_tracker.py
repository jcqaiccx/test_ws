#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Person detection and tracking skeleton for the HRI receptionist task.

Scope:
- Subscribe to camera images.
- Maintain a tracked target.
- Prefer the person near the door region when selecting the current guest.
- Publish the current target for the task manager.

This file is intentionally lightweight: the actual detector can be replaced
with any model (YOLO, OpenPose, MediaPipe, re-id tracker, etc.).
"""

from dataclasses import dataclass, asdict
from typing import List, Optional, Tuple
import json
import math
import ast

import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2


@dataclass
class PersonDetection:
    track_id: int
    cx: float
    cy: float
    w: float
    h: float
    confidence: float
    label: str = "person"


@dataclass
class TrackedTarget:
    track_id: int = -1
    cx: float = -1.0
    cy: float = -1.0
    w: float = -1.0
    h: float = -1.0
    confidence: float = 0.0
    selected_reason: str = "none"


class PersonTrackerNode:
    def __init__(self):
        rospy.init_node("person_tracker", anonymous=False)

        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/rgb/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/rgb/camera_info")
        self.door_roi = rospy.get_param("~door_roi", [0.65, 0.25, 0.35, 0.50])
        if isinstance(self.door_roi, str):
            try:
                self.door_roi = ast.literal_eval(self.door_roi)
            except (ValueError, SyntaxError):
                rospy.logwarn("[person_tracker] invalid door_roi string: %s, fallback to default", self.door_roi)
                self.door_roi = [0.65, 0.25, 0.35, 0.50]
        if not isinstance(self.door_roi, (list, tuple)) or len(self.door_roi) != 4:
            rospy.logwarn("[person_tracker] invalid door_roi format: %s, fallback to default", self.door_roi)
            self.door_roi = [0.65, 0.25, 0.35, 0.50]
        self.door_roi = [float(v) for v in self.door_roi]
        # door_roi is normalized: [x_min, y_min, width, height]
        self.min_confidence = rospy.get_param("~min_confidence", 0.40)
        self.publish_rate = rospy.get_param("~publish_rate", 10)

        self.frame_width = None
        self.frame_height = None
        self.camera_info_received = False
        self.bridge = CvBridge()

        # OpenCV HOG person detector.
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self.detector_stride = rospy.get_param("~hog_stride", 8)
        self.detector_padding = rospy.get_param("~hog_padding", 8)
        self.detector_scale = rospy.get_param("~hog_scale", 1.05)

        # Simple track-id assignment state.
        self.prev_tracks: List[PersonDetection] = []
        self.next_track_id = 1
        self.max_track_distance = rospy.get_param("~max_track_distance", 0.10)
        self.max_lost_frames = rospy.get_param("~max_lost_frames", 8)
        self.lost_frames = 0

        self.current_target = TrackedTarget()
        self.last_detections: List[PersonDetection] = []
        self.target_locked = False

        self.current_target_pub = rospy.Publisher("/person_tracker/current_target", String, queue_size=10)
        self.targets_pub = rospy.Publisher("/person_tracker/targets", String, queue_size=10)
        self.debug_pub = rospy.Publisher("/person_tracker/debug_text", String, queue_size=10)

        self.rgb_sub = rospy.Subscriber(self.rgb_topic, Image, self.rgb_callback, queue_size=1)
        self.info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=1)
        self.reset_sub = rospy.Subscriber("/person_tracker/reset", String, self.reset_callback, queue_size=10)
        self.lock_sub = rospy.Subscriber("/person_tracker/select_target", String, self.select_target_callback, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1.0 / float(self.publish_rate)), self.publish_loop)

    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.frame_width = int(msg.width)
            self.frame_height = int(msg.height)
            self.camera_info_received = True
            rospy.loginfo("[person_tracker] camera info received: %sx%s", self.frame_width, self.frame_height)

    def reset_callback(self, msg: String):
        self.current_target = TrackedTarget()
        self.target_locked = False
        self.lost_frames = 0
        self.debug("Tracker reset")

    def select_target_callback(self, msg: String):
        # Payload can be either "door" or a specific track id.
        if msg.data.strip().lower() == "door":
            self.target_locked = False
            self.lost_frames = 0
            self.debug("Target selection requested: door region")
            self.select_best_target(prefer_door=True)
            return

        try:
            wanted_id = int(msg.data.strip())
        except ValueError:
            self.debug(f"Invalid target selection request: {msg.data}")
            return

        for det in self.last_detections:
            if det.track_id == wanted_id:
                self.assign_target(det, reason="manual_id")
                self.target_locked = True
                self.lost_frames = 0
                self.debug(f"Target locked by id: {wanted_id}")
                return

        self.debug(f"Requested target id not found: {wanted_id}")

    def debug(self, text: str):
        rospy.loginfo("[person_tracker] %s", text)
        self.debug_pub.publish(String(data=text))

    def rgb_callback(self, msg: Image):
        # Placeholder: replace this with your detector's output.
        # The tracker currently expects detections to be provided by
        # the `detect_people` function.
        detections = self.detect_people(msg)
        self.last_detections = detections

        if not detections:
            # Keep current target if the lock is still valid.
            self.publish_targets([])
            self.handle_no_detections()
            return

        if not self.target_locked:
            self.select_best_target(prefer_door=True)
        else:
            self.update_locked_target(detections)

        self.publish_targets(detections)

    def detect_people(self, msg: Image) -> List[PersonDetection]:
        """Return detected people in the current frame.

        Replace this method with your actual detector. The expected output is a
        list of PersonDetection objects with normalized coordinates [0, 1].
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logwarn_throttle(2.0, "[person_tracker] cv_bridge conversion failed: %s", e)
            return []

        h, w = frame.shape[:2]
        if h <= 0 or w <= 0:
            return []

        # HOG returns rectangles in pixel coordinates.
        rects, weights = self.hog.detectMultiScale(
            frame,
            winStride=(self.detector_stride, self.detector_stride),
            padding=(self.detector_padding, self.detector_padding),
            scale=self.detector_scale,
        )

        raw_dets = []
        for i, (x, y, bw, bh) in enumerate(rects):
            conf = float(weights[i]) if i < len(weights) else 0.0
            if conf < self.min_confidence:
                continue

            cx = (x + bw * 0.5) / float(w)
            cy = (y + bh * 0.5) / float(h)
            nw = bw / float(w)
            nh = bh / float(h)
            raw_dets.append((cx, cy, nw, nh, conf))

        detections = self.assign_track_ids(raw_dets)
        self.prev_tracks = detections
        return detections

    def assign_track_ids(self, raw_dets: List[Tuple[float, float, float, float, float]]) -> List[PersonDetection]:
        if not raw_dets:
            return []

        assigned: List[PersonDetection] = []
        used_prev_ids = set()

        for (cx, cy, bw, bh, conf) in raw_dets:
            best_prev: Optional[PersonDetection] = None
            best_dist = 1e9

            for prev in self.prev_tracks:
                if prev.track_id in used_prev_ids:
                    continue
                dist = math.hypot(cx - prev.cx, cy - prev.cy)
                if dist < best_dist:
                    best_dist = dist
                    best_prev = prev

            if best_prev is not None and best_dist <= self.max_track_distance:
                track_id = best_prev.track_id
                used_prev_ids.add(track_id)
            else:
                track_id = self.next_track_id
                self.next_track_id += 1

            assigned.append(
                PersonDetection(
                    track_id=track_id,
                    cx=cx,
                    cy=cy,
                    w=bw,
                    h=bh,
                    confidence=conf,
                )
            )

        return assigned

    def inside_door_roi(self, det: PersonDetection) -> bool:
        x_min, y_min, roi_w, roi_h = self.door_roi
        x_max = x_min + roi_w
        y_max = y_min + roi_h
        x = det.cx
        y = det.cy
        return (x_min <= x <= x_max) and (y_min <= y <= y_max)

    def select_best_target(self, prefer_door: bool = True):
        if not self.last_detections:
            return

        candidates = [d for d in self.last_detections if d.confidence >= self.min_confidence]
        if not candidates:
            return

        if prefer_door:
            door_candidates = [d for d in candidates if self.inside_door_roi(d)]
            if door_candidates:
                best = max(door_candidates, key=lambda d: d.confidence)
                self.assign_target(best, reason="door_roi")
                self.target_locked = True
                return

        # Fallback: choose the most confident detection, then the one closest to
        # the door region center.
        roi_center = (self.door_roi[0] + self.door_roi[2] / 2.0, self.door_roi[1] + self.door_roi[3] / 2.0)

        def score(det: PersonDetection):
            dist = math.hypot(det.cx - roi_center[0], det.cy - roi_center[1])
            return (det.confidence, -dist)

        best = max(candidates, key=score)
        self.assign_target(best, reason="best_confidence")
        self.target_locked = True

    def update_locked_target(self, detections: List[PersonDetection]):
        if self.current_target.track_id < 0:
            self.target_locked = False
            return

        for det in detections:
            if det.track_id == self.current_target.track_id:
                self.assign_target(det, reason="track_update")
                self.lost_frames = 0
                return

        # If the locked target disappears, tolerate a few missed frames before switching.
        self.lost_frames += 1
        self.debug(f"Locked target lost: {self.current_target.track_id} (lost_frames={self.lost_frames})")
        if self.lost_frames >= self.max_lost_frames:
            self.current_target = TrackedTarget()
            self.target_locked = False
            self.lost_frames = 0
            self.select_best_target(prefer_door=False)

    def handle_no_detections(self):
        if self.current_target.track_id < 0:
            return

        self.lost_frames += 1
        self.debug(f"No detections, keeping target {self.current_target.track_id} (lost_frames={self.lost_frames})")
        if self.lost_frames >= self.max_lost_frames:
            self.debug(f"Locked target timed out: {self.current_target.track_id}")
            self.current_target = TrackedTarget()
            self.target_locked = False
            self.lost_frames = 0

    def assign_target(self, det: PersonDetection, reason: str):
        self.current_target = TrackedTarget(
            track_id=det.track_id,
            cx=det.cx,
            cy=det.cy,
            w=det.w,
            h=det.h,
            confidence=det.confidence,
            selected_reason=reason,
        )

    def publish_targets(self, detections: List[PersonDetection]):
        payload = {
            "current_target": asdict(self.current_target),
            "detection_count": len(detections),
            "target_locked": self.target_locked,
            "door_roi": self.door_roi,
        }
        self.current_target_pub.publish(String(data=json.dumps(payload)))
        self.targets_pub.publish(String(data=json.dumps([asdict(d) for d in detections])))

    def publish_loop(self, _event):
        # Keep publishing the latest target so that downstream modules can read it.
        if self.current_target.track_id >= 0:
            self.debug(f"target={self.current_target.track_id} reason={self.current_target.selected_reason}")
        else:
            self.debug("no target")


def main():
    node = PersonTrackerNode()
    rospy.loginfo("[person_tracker] node started, rgb_topic=%s, camera_info_topic=%s",
                  node.rgb_topic, node.camera_info_topic)
    rospy.spin()


if __name__ == "__main__":
    main()
