#!/usr/bin/env python3
import json
import os
import urllib.request

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


MODEL_FILES = {
    "deploy.prototxt": "https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/deploy.prototxt",
    "mobilenet_iter_73000.caffemodel": "https://github.com/chuanqi305/MobileNet-SSD/raw/master/mobilenet_iter_73000.caffemodel",
}


def ensure_model_files(model_dir):
    os.makedirs(model_dir, exist_ok=True)
    local_paths = {}
    for filename, url in MODEL_FILES.items():
        dst = os.path.join(model_dir, filename)
        if not os.path.exists(dst):
            try:
                urllib.request.urlretrieve(url, dst)
            except Exception:
                pass
        local_paths[filename] = dst
    return local_paths

CLASS_NAMES = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat",
    "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant",
    "sheep", "sofa", "train", "tvmonitor"
]


class ObjectShelfDetectorNode:
    def __init__(self):
        rospy.init_node("object_shelf_detector")
        self.bridge = CvBridge()

        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_color")
        self.score_threshold = float(rospy.get_param("~score_threshold", 0.4))
        self.max_objects = int(rospy.get_param("~max_objects", 5))
        self.object_topic = rospy.get_param("~object_topic", "/vision/object_detections")
        self.vision_back_topic = rospy.get_param("~vision_back_topic", "vision_back")
        self.debug_topic = rospy.get_param("~debug_topic", "/vision/object_debug")
        self.publish_debug = bool(rospy.get_param("~publish_debug", True))

        target_class_csv = rospy.get_param("~target_classes", "bottle,chair,tvmonitor,pottedplant")
        self.target_classes = {name.strip() for name in target_class_csv.split(",") if name.strip()}

        package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        model_dir = rospy.get_param("~model_dir", os.path.join(package_root, "models"))

        self.net = None
        local_paths = ensure_model_files(model_dir)
        prototxt = local_paths["deploy.prototxt"]
        model = local_paths["mobilenet_iter_73000.caffemodel"]
        if os.path.exists(prototxt) and os.path.exists(model):
            try:
                self.net = cv2.dnn.readNetFromCaffe(prototxt, model)
            except Exception as exc:
                rospy.logwarn("Failed to load object detection model: %s", exc)
        if self.net is None:
            rospy.logwarn("object_shelf_detector running without model; publishing empty detections")

        self.object_pub = rospy.Publisher(self.object_topic, String, queue_size=10)
        self.vision_back_pub = rospy.Publisher(self.vision_back_topic, String, queue_size=10)
        self.debug_pub = rospy.Publisher(self.debug_topic, Image, queue_size=1)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)

        rospy.loginfo("object_shelf_detector started: image=%s object_topic=%s", self.image_topic, self.object_topic)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "CV bridge conversion failed: %s", exc)
            return

        h, w = frame.shape[:2]
        detections = None
        if self.net is not None:
            blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)
            self.net.setInput(blob)
            detections = self.net.forward()

        objects = []
        if detections is not None:
            for i in range(detections.shape[2]):
                score = float(detections[0, 0, i, 2])
                if score < self.score_threshold:
                    continue

                cls_idx = int(detections[0, 0, i, 1])
                if cls_idx >= len(CLASS_NAMES):
                    continue
                label = CLASS_NAMES[cls_idx]
                if self.target_classes and label not in self.target_classes:
                    continue

                box = detections[0, 0, i, 3:7] * [w, h, w, h]
                x1, y1, x2, y2 = box.astype(int)
                x1 = max(0, min(x1, w - 1))
                y1 = max(0, min(y1, h - 1))
                x2 = max(0, min(x2, w - 1))
                y2 = max(0, min(y2, h - 1))
                bw = max(1, x2 - x1)
                bh = max(1, y2 - y1)
                cx_norm = float((x1 + x2) / 2.0 / w)

                side = "center"
                if cx_norm < 0.4:
                    side = "left"
                elif cx_norm > 0.6:
                    side = "right"

                objects.append({
                    "label": label,
                    "score": score,
                    "bbox": [int(x1), int(y1), int(bw), int(bh)],
                    "cx": cx_norm,
                    "cy": float((y1 + y2) / 2.0 / h),
                    "side": side,
                })

        objects.sort(key=lambda x: x["score"], reverse=True)
        objects = objects[: self.max_objects]

        payload = {
            "stamp": msg.header.stamp.to_sec(),
            "frame_id": msg.header.frame_id,
            "count": len(objects),
            "objects": objects,
        }
        self.object_pub.publish(String(data=json.dumps(payload, ensure_ascii=True)))

        if objects:
            best = objects[0]
            # Keep backward-compatible side message used by existing state machine.
            self.vision_back_pub.publish(String(data=best["side"]))
            # Also publish explicit recognized object for debugging/integration.
            self.vision_back_pub.publish(
                String(data="object:{}:{}:{:.2f}".format(best["label"], best["side"], best["score"]))
            )

        if self.publish_debug:
            for obj in objects:
                x, y, bw, bh = obj["bbox"]
                cv2.rectangle(frame, (x, y), (x + bw, y + bh), (30, 160, 255), 2)
                cv2.putText(
                    frame,
                    "{} {:.2f} {}".format(obj["label"], obj["score"], obj["side"]),
                    (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (30, 160, 255),
                    2,
                )
            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
            except Exception:
                pass


if __name__ == "__main__":
    node = ObjectShelfDetectorNode()
    rospy.spin()
