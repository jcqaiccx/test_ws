#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import random

import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent, CliffEvent
from sensor_msgs.msg import LaserScan


class AutoExploreMapping:
    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/scan_filtered")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel_mux/input/navi")
        self.forward_speed = float(rospy.get_param("~forward_speed", 0.12))
        self.turn_speed = float(rospy.get_param("~turn_speed", 0.7))
        self.safe_distance = float(rospy.get_param("~safe_distance", 0.55))
        self.turn_hold_sec = float(rospy.get_param("~turn_hold_sec", 1.0))
        self.reverse_speed = float(rospy.get_param("~reverse_speed", 0.08))
        self.scan_timeout_sec = float(rospy.get_param("~scan_timeout_sec", 0.8))

        self.bumper_topic = rospy.get_param("~bumper_topic", "/mobile_base/events/bumper")
        self.cliff_topic = rospy.get_param("~cliff_topic", "/mobile_base/events/cliff")

        self.latest_scan = None
        self.turn_until = rospy.Time(0)
        self.hazard_until = rospy.Time(0)
        self.hazard_turn_sign = 1.0
        self.last_scan_wall_time = rospy.Time(0)

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=5)
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber(self.bumper_topic, BumperEvent, self.bumper_callback, queue_size=5)
        rospy.Subscriber(self.cliff_topic, CliffEvent, self.cliff_callback, queue_size=5)
        rospy.on_shutdown(self.stop_robot)

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.last_scan_wall_time = rospy.Time.now()

    def bumper_callback(self, msg):
        if msg.state != BumperEvent.PRESSED:
            return
        # If right bumper is pressed, rotate left; otherwise rotate right.
        self.hazard_turn_sign = 1.0 if msg.bumper == BumperEvent.RIGHT else -1.0
        self.hazard_until = rospy.Time.now() + rospy.Duration(1.2)

    def cliff_callback(self, msg):
        if msg.state != CliffEvent.CLIFF:
            return
        # Back off and rotate away from the triggered cliff side.
        self.hazard_turn_sign = 1.0 if msg.sensor == CliffEvent.RIGHT else -1.0
        self.hazard_until = rospy.Time.now() + rospy.Duration(1.3)

    def _sector_min(self, scan, center_deg, width_deg):
        start = math.radians(center_deg - width_deg / 2.0)
        end = math.radians(center_deg + width_deg / 2.0)
        values = []
        angle = scan.angle_min
        for r in scan.ranges:
            if start <= angle <= end and math.isfinite(r) and r > 0.01:
                values.append(r)
            angle += scan.angle_increment
        return min(values) if values else float("inf")

    def decide_twist(self):
        tw = Twist()
        if self.latest_scan is None:
            return tw

        now = rospy.Time.now()

        # Emergency behavior after bumper/cliff trigger.
        if now < self.hazard_until:
            tw.linear.x = -abs(self.reverse_speed)
            tw.angular.z = self.hazard_turn_sign * abs(self.turn_speed)
            return tw

        # Fail-safe: if scan stream stalls, stop advancing.
        if self.last_scan_wall_time != rospy.Time(0):
            if (now - self.last_scan_wall_time).to_sec() > self.scan_timeout_sec:
                tw.linear.x = 0.0
                tw.angular.z = self.turn_speed * 0.6
                return tw

        if now < self.turn_until:
            tw.angular.z = self.turn_speed
            return tw

        front = self._sector_min(self.latest_scan, 0.0, 50.0)
        left = self._sector_min(self.latest_scan, 60.0, 60.0)
        right = self._sector_min(self.latest_scan, -60.0, 60.0)

        if front < self.safe_distance:
            # Rotate away from the closer side to escape obstacles.
            tw.linear.x = 0.0
            tw.angular.z = self.turn_speed if left > right else -self.turn_speed
            self.turn_until = now + rospy.Duration(self.turn_hold_sec + random.uniform(0.2, 0.8))
            return tw

        tw.linear.x = self.forward_speed
        tw.angular.z = random.uniform(-0.12, 0.12)
        return tw

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.decide_twist())
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("auto_explore_mapping", anonymous=False)
    AutoExploreMapping().run()
