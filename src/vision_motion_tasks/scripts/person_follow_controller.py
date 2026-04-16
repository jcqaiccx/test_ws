#!/usr/bin/env python3
import json
import math

import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String


class PersonFollowControllerNode:
    def __init__(self):
        rospy.init_node("person_follow_controller")
        self.bridge = CvBridge()

        self.person_topic = rospy.get_param("~person_topic", "/vision/person_target")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel_mux/input/navi")

        self.pid_topic = rospy.get_param("~pid_topic", "pid")
        self.pid_back_topic = rospy.get_param("~pid_back_topic", "pid_back")

        self.kp_lin = float(rospy.get_param("~kp_lin", 0.7))
        self.kp_ang = float(rospy.get_param("~kp_ang", 1.8))
        self.max_lin = float(rospy.get_param("~max_lin", 0.35))
        self.min_lin = float(rospy.get_param("~min_lin", 0.08))
        self.max_ang = float(rospy.get_param("~max_ang", 1.0))
        self.invert_linear = bool(rospy.get_param("~invert_linear", False))
        self.goal_distance = float(rospy.get_param("~goal_distance", 1.0))
        self.stop_distance = float(rospy.get_param("~stop_distance", 0.65))
        self.target_bbox_h_ratio = float(rospy.get_param("~target_bbox_h_ratio", 0.45))
        self.stop_bbox_h_ratio = float(rospy.get_param("~stop_bbox_h_ratio", 0.55))
        self.kp_bbox = float(rospy.get_param("~kp_bbox", 0.9))
        self.lost_timeout = float(rospy.get_param("~lost_timeout", 1.0))
        self.deadband = float(rospy.get_param("~deadband", 0.03))

        self.follow_enabled = bool(rospy.get_param("~start_enabled", False))

        self.last_person_stamp = rospy.Time(0)
        self.last_person = None
        self.last_depth = None

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.pid_back_pub = rospy.Publisher(self.pid_back_topic, String, queue_size=10)

        rospy.Subscriber(self.pid_topic, String, self.pid_command_callback, queue_size=10)
        rospy.Subscriber(self.person_topic, String, self.person_callback, queue_size=10)
        rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=1, buff_size=2**24)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)
        rospy.loginfo("person_follow_controller started: person=%s depth=%s", self.person_topic, self.depth_topic)

    def pid_command_callback(self, msg):
        cmd = (msg.data or "").strip().lower()
        if cmd == "follow_people":
            self.follow_enabled = True
        elif cmd in {"pid_stop", "stop_follow"}:
            self.follow_enabled = False
            self.cmd_pub.publish(Twist())
            self.pid_back_pub.publish(String(data="follow_people_finish"))

    def person_callback(self, msg):
        try:
            self.last_person = json.loads(msg.data)
            self.last_person_stamp = rospy.Time.now()
        except Exception:
            pass

    def depth_callback(self, msg):
        try:
            if msg.encoding in ("16UC1", "mono16"):
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough").astype(np.float32)
                img = img / 1000.0
            else:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough").astype(np.float32)
            self.last_depth = img
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Depth conversion failed: %s", exc)

    def estimate_distance(self, bbox):
        if self.last_depth is None or bbox is None:
            return None

        h, w = self.last_depth.shape[:2]
        x, y, bw, bh = bbox
        cx = int(max(0, min(w - 1, x + bw / 2.0)))
        cy = int(max(0, min(h - 1, y + bh / 2.0)))

        win = 8
        x1 = max(0, cx - win)
        x2 = min(w, cx + win)
        y1 = max(0, cy - win)
        y2 = min(h, cy + win)
        patch = self.last_depth[y1:y2, x1:x2]

        valid = patch[np.isfinite(patch)]
        valid = valid[(valid > 0.15) & (valid < 6.0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def control_loop(self, _event):
        if not self.follow_enabled:
            return

        if self.last_person is None or (rospy.Time.now() - self.last_person_stamp).to_sec() > self.lost_timeout:
            self.cmd_pub.publish(Twist())
            return

        if not bool(self.last_person.get("found", False)):
            self.cmd_pub.publish(Twist())
            return

        cx = float(self.last_person.get("cx", 0.5))
        bbox = self.last_person.get("bbox", None)
        bbox_h_ratio = float(self.last_person.get("bbox_height_ratio", 0.0))

        distance = None
        if isinstance(bbox, list) and len(bbox) == 4:
            distance = self.estimate_distance([int(v) for v in bbox])

        err_ang = 0.5 - cx
        if abs(err_ang) < self.deadband:
            err_ang = 0.0

        cmd = Twist()
        cmd.angular.z = max(-self.max_ang, min(self.max_ang, self.kp_ang * err_ang))

        if distance is not None:
            if distance < self.stop_distance:
                cmd.linear.x = 0.0
            else:
                err_lin = distance - self.goal_distance
                if err_lin > 0:
                    cmd.linear.x = max(self.min_lin, min(self.max_lin, self.kp_lin * err_lin))
                else:
                    cmd.linear.x = 0.0
        else:
            # Depth can be unavailable or misaligned; fallback to apparent person size.
            if bbox_h_ratio >= self.stop_bbox_h_ratio:
                cmd.linear.x = 0.0
            elif bbox_h_ratio > 0.0:
                err_bbox = max(0.0, self.target_bbox_h_ratio - bbox_h_ratio)
                if err_bbox > 0.0:
                    cmd.linear.x = max(self.min_lin, min(self.max_lin, self.kp_bbox * err_bbox))
                else:
                    cmd.linear.x = 0.0
            else:
                cmd.linear.x = 0.0

        if self.invert_linear:
            cmd.linear.x = -cmd.linear.x

        self.cmd_pub.publish(cmd)


if __name__ == "__main__":
    node = PersonFollowControllerNode()
    rospy.spin()
