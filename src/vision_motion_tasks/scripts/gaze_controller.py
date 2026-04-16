#!/usr/bin/env python3
import json

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class GazeControllerNode:
    def __init__(self):
        rospy.init_node("gaze_controller")

        self.person_topic = rospy.get_param("~person_topic", "/vision/person_target")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel_mux/input/navi")
        self.vision_cmd_topic = rospy.get_param("~vision_cmd_topic", "vision")
        self.vision_back_topic = rospy.get_param("~vision_back_topic", "vision_back")

        self.kp_yaw = float(rospy.get_param("~kp_yaw", 3.4))
        self.max_ang = float(rospy.get_param("~max_ang", 1.8))
        self.min_ang = float(rospy.get_param("~min_ang", 0.55))
        self.target_cx = float(rospy.get_param("~target_cx", 0.5))
        self.lost_timeout = float(rospy.get_param("~lost_timeout", 0.8))
        self.deadband = float(rospy.get_param("~deadband", 0.012))
        self.err_alpha = float(rospy.get_param("~err_alpha", 0.7))
        self.lost_hold_sec = float(rospy.get_param("~lost_hold_sec", 0.35))
        self.invert_angular = bool(rospy.get_param("~invert_angular", False))
        self.response_power = float(rospy.get_param("~response_power", 0.8))
        self.big_error_threshold = float(rospy.get_param("~big_error_threshold", 0.2))
        self.big_error_boost = float(rospy.get_param("~big_error_boost", 1.3))

        self.enabled = bool(rospy.get_param("~start_enabled", True))
        self.last_person_stamp = rospy.Time(0)
        self.last_person = None
        self.last_state_report = ""
        self.filtered_err = 0.0
        self.last_cmd_ang = 0.0
        self.last_cmd_stamp = rospy.Time(0)

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.vision_back_pub = rospy.Publisher(self.vision_back_topic, String, queue_size=10)

        rospy.Subscriber(self.person_topic, String, self.person_callback, queue_size=10)
        rospy.Subscriber(self.vision_cmd_topic, String, self.command_callback, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)
        rospy.loginfo("gaze_controller started: person=%s cmd_vel=%s", self.person_topic, self.cmd_vel_topic)

    def command_callback(self, msg):
        cmd = (msg.data or "").strip().lower()
        if cmd in {"start_gaze", "gaze_on", "look_people", "look_person"}:
            self.enabled = True
        elif cmd in {"stop_gaze", "gaze_off"}:
            self.enabled = False
            self.publish_stop()

    def person_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self.last_person = data
        self.last_person_stamp = rospy.Time.now()

    def publish_stop(self):
        self.last_cmd_ang = 0.0
        self.cmd_pub.publish(Twist())

    def publish_turn(self, ang_z):
        cmd = Twist()
        cmd.angular.z = float(ang_z)
        self.cmd_pub.publish(cmd)
        self.last_cmd_ang = float(ang_z)
        self.last_cmd_stamp = rospy.Time.now()

    def set_state(self, state_text):
        if state_text != self.last_state_report:
            self.vision_back_pub.publish(String(data=state_text))
            self.last_state_report = state_text

    def control_loop(self, _event):
        if not self.enabled:
            return

        now = rospy.Time.now()
        age = (now - self.last_person_stamp).to_sec()
        has_person = self.last_person is not None and bool(self.last_person.get("found", False)) and age <= self.lost_timeout

        if not has_person:
            # Briefly keep turning after momentary frame drops.
            if self.last_cmd_ang != 0.0 and (now - self.last_cmd_stamp).to_sec() <= self.lost_hold_sec:
                self.publish_turn(self.last_cmd_ang * 0.75)
                self.set_state("gaze_tracking")
            else:
                self.publish_stop()
                self.set_state("gaze_lost")
            return

        cx = float(self.last_person.get("cx", self.target_cx))
        err = self.target_cx - cx
        self.filtered_err = self.err_alpha * err + (1.0 - self.err_alpha) * self.filtered_err

        use_err = self.filtered_err
        if self.invert_angular:
            use_err = -use_err

        if abs(use_err) < self.deadband:
            self.publish_stop()
            self.set_state("gaze_locked")
            return

        mag = abs(use_err)
        shaped = pow(mag, self.response_power)
        if mag > self.big_error_threshold:
            shaped *= self.big_error_boost

        ang = self.kp_yaw * shaped
        if use_err < 0.0:
            ang = -ang
        ang = max(-self.max_ang, min(self.max_ang, ang))
        if abs(ang) < self.min_ang:
            ang = self.min_ang if ang > 0.0 else -self.min_ang

        self.publish_turn(ang)
        self.set_state("gaze_tracking")


if __name__ == "__main__":
    node = GazeControllerNode()
    rospy.spin()
