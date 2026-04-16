#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import subprocess
import threading

import actionlib
import rospy
import tf
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Empty, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker


ACTION_MOVE_BASE = "move_base"
FRAME_MAP = "map"


class NavToPoint:
    def __init__(self):
        rospy.on_shutdown(self.clean_up)

        self.goal_topic = rospy.get_param("~goal_topic", "navigation")
        self.feedback_topic = rospy.get_param("~feedback_topic", "navigation_back")

        self.goal_pose = (
            float(rospy.get_param("~goal_x", 1.0)),
            float(rospy.get_param("~goal_y", 0.0)),
            float(rospy.get_param("~goal_yaw", 0.0)),
        )
        self.home_pose = (
            float(rospy.get_param("~home_x", 0.0)),
            float(rospy.get_param("~home_y", 0.0)),
            float(rospy.get_param("~home_yaw", 0.0)),
        )

        self.auto_start = rospy.get_param("~auto_start", True)
        self.publish_initial_pose_on_start = rospy.get_param("~publish_initial_pose_on_start", False)
        self.initial_pose = (
            float(rospy.get_param("~initial_pose_x", self.home_pose[0])),
            float(rospy.get_param("~initial_pose_y", self.home_pose[1])),
            float(rospy.get_param("~initial_pose_yaw", self.home_pose[2])),
        )

        self.save_map_on_goal = rospy.get_param("~save_map_on_goal", True)
        self.save_map_on_return = rospy.get_param("~save_map_on_return", False)
        self.stop_mapping_after_goal = rospy.get_param("~stop_mapping_after_goal", True)
        self.gmapping_node_name = rospy.get_param("~gmapping_node_name", "/slam_gmapping")
        self.map_base_path = rospy.get_param(
            "~map_base_path",
            os.path.join(os.path.expanduser("~"), "catkin_ws", "src", "navigation", "maps", "online_map"),
        )

        self.goal_lock = threading.Lock()
        self.busy = False
        self.start_pose = None
        self.last_result = None
        self.last_dynamic_goal = None

        self.move_base = actionlib.SimpleActionClient(ACTION_MOVE_BASE, MoveBaseAction)
        rospy.loginfo("Waiting for move_base server...")
        if not self.move_base.wait_for_server(rospy.Duration(60.0)):
            rospy.logwarn("move_base server not ready yet; navigation will keep retrying through commands.")
        else:
            rospy.loginfo("Connected to move_base server.")

        self.listener = tf.TransformListener()
        self.pose_marker_pub = rospy.Publisher("/robot_pose_marker", Marker, queue_size=1)
        self.goal_marker_pub = rospy.Publisher("/goal_point_marker", Marker, queue_size=1)
        self.initial_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.feedback_pub = rospy.Publisher(self.feedback_topic, String, queue_size=10)
        rospy.Subscriber(self.goal_topic, String, self.navigation_command_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.rviz_goal_callback)
        rospy.Subscriber("/stop_signal", Empty, self.stop_and_save_callback)

        if self.publish_initial_pose_on_start:
            self.publish_initial_pose(self.initial_pose)

        if self.auto_start:
            threading.Thread(target=self._handle_command, args=("start_nav",), daemon=True).start()

        rospy.sleep(1.0)

    def navigation_command_callback(self, msg):
        command = (msg.data or "").strip()
        if not command:
            return
        threading.Thread(target=self._handle_command, args=(command,), daemon=True).start()

    def _handle_command(self, command):
        if command in {"start_nav", "nav_start", "goto_goal"}:
            target = self.last_dynamic_goal or self.goal_pose
            self.navigate_to_goal(target, save_map=self.save_map_on_goal)
            return
        if command in {"nav_back", "return_home", "go_home"}:
            target = self.start_pose or self.home_pose
            self.navigate_to_goal(target, save_map=self.save_map_on_return)
            return
        if command in {"save_map", "map_save"}:
            self.save_map()
            return
        if command in {"finish_mapping", "mapping_done", "save_and_stop_map"}:
            self.save_map()
            self.stop_mapping()
            self.feedback_pub.publish(String(data="map_saved"))
            return
        if command in {"stop", "cancel"}:
            self.stop_current_goal()
            return
        rospy.loginfo("Ignored navigation command: %s", command)

    def rviz_goal_callback(self, msg):
        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.last_dynamic_goal = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            math.degrees(yaw),
        )
        rospy.loginfo(
            "Received RViz goal: x=%.3f y=%.3f yaw=%.1f",
            self.last_dynamic_goal[0],
            self.last_dynamic_goal[1],
            self.last_dynamic_goal[2],
        )

        # If idle, execute the new RViz goal immediately.
        with self.goal_lock:
            is_busy = self.busy
        if not is_busy:
            threading.Thread(
                target=self.navigate_to_goal,
                args=(self.last_dynamic_goal, self.save_map_on_goal),
                daemon=True,
            ).start()

    def navigate_to_goal(self, target, save_map=False):
        with self.goal_lock:
            if self.busy:
                rospy.logwarn("Navigation is already running; new command ignored.")
                return
            self.busy = True

        try:
            if self.start_pose is None:
                current_pose = self.get_current_pose()
                if current_pose is not None:
                    self.start_pose = current_pose

            success = self.goto(target, blocking=True)
            if success:
                if save_map:
                    self.save_map()
                    if self.stop_mapping_after_goal:
                        self.stop_mapping()
                self.last_result = "nav_complete"
                self.feedback_pub.publish(String(data="nav_complete"))
            else:
                self.last_result = "nav_failed"
                self.feedback_pub.publish(String(data="nav_failed"))
        finally:
            with self.goal_lock:
                self.busy = False

    def goto(self, target, blocking=True):
        if not self.move_base.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("move_base server is unavailable.")
            return False

        yaw = math.radians(float(target[2]))
        quaternion = quaternion_from_euler(0.0, 0.0, yaw)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = FRAME_MAP
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(
            Point(float(target[0]), float(target[1]), 0.0),
            Quaternion(*quaternion),
        )

        self.publish_goal_marker(target[0], target[1], quaternion)
        rospy.loginfo("Sending goal: x=%.3f y=%.3f yaw=%.1f", float(target[0]), float(target[1]), float(target[2]))
        self.move_base.send_goal(goal)

        if not blocking:
            return True

        success = self.move_base.wait_for_result()
        if not success:
            rospy.logerr("Failed to reach goal before timeout or cancellation.")
            return False

        result = self.move_base.get_result()
        rospy.loginfo("Navigation result: %s", result)
        return True

    def get_current_pose(self):
        try:
            trans, rot = self.listener.lookupTransform(FRAME_MAP, "base_link", rospy.Time(0))
            yaw = euler_from_quaternion(rot)[2]
            return (float(trans[0]), float(trans[1]), math.degrees(yaw))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Cannot get robot pose from TF yet.")
            return None

    def publish_initial_pose(self, pose):
        x, y, yaw_deg = pose
        quat = quaternion_from_euler(0.0, 0.0, math.radians(float(yaw_deg)))
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = FRAME_MAP
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        msg.pose.covariance = [
            0.5, 0, 0, 0, 0, 0,
            0, 0.5, 0, 0, 0, 0,
            0, 0, 0.5, 0, 0, 0,
            0, 0, 0, 0.5, 0, 0,
            0, 0, 0, 0, 0.5, 0,
            0, 0, 0, 0, 0, 0.068,
        ]
        for _ in range(3):
            self.initial_pose_pub.publish(msg)
            rospy.sleep(0.2)

    def publish_goal_marker(self, x, y, quat):
        marker = Marker()
        marker.header.frame_id = FRAME_MAP
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal_point"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.15
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        marker.scale.x = 0.45
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.a = 1.0
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.2
        self.goal_marker_pub.publish(marker)

    def save_map(self):
        base_path = os.path.splitext(self.map_base_path)[0]
        os.makedirs(os.path.dirname(base_path), exist_ok=True)
        rospy.loginfo("Saving map to %s.[yaml|pgm]", base_path)
        try:
            subprocess.run(
                ["rosrun", "map_server", "map_saver", "-f", base_path],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            rospy.loginfo("Map saved successfully.")
        except (subprocess.CalledProcessError, FileNotFoundError) as exc:
            rospy.logerr("Failed to save map: %s", exc)

    def stop_mapping(self):
        try:
            subprocess.run(
                ["rosnode", "kill", self.gmapping_node_name],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            rospy.loginfo("Stopped mapping node: %s", self.gmapping_node_name)
        except (subprocess.CalledProcessError, FileNotFoundError) as exc:
            rospy.logwarn("Cannot stop mapping node %s: %s", self.gmapping_node_name, exc)

    def stop_and_save_callback(self, _msg):
        self.stop_current_goal()
        self.save_map()

    def stop_current_goal(self):
        with self.goal_lock:
            self.move_base.cancel_all_goals()
        rospy.loginfo("Current navigation goal cancelled.")

    def clean_up(self):
        try:
            self.stop_current_goal()
        except Exception:
            pass


def main():
    rospy.init_node("navigation_controller", anonymous=False)
    NavToPoint()
    rospy.spin()


if __name__ == "__main__":
    main()
