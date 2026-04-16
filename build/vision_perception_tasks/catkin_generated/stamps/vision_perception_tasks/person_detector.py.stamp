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
                # Offline environments are common on robots; caller will fallback.
                pass
        local_paths[filename] = dst
    return local_paths

CLASS_NAMES = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat",
    "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant",
    "sheep", "sofa", "train", "tvmonitor"
]


class PersonDetectorNode:
    def __init__(self):
        rospy.init_node("person_detector")
        self.bridge = CvBridge()

        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_color")
        self.score_threshold = float(rospy.get_param("~score_threshold", 0.45))
        self.person_topic = rospy.get_param("~person_topic", "/vision/person_target")
        self.debug_topic = rospy.get_param("~debug_topic", "/vision/person_debug")
        self.publish_debug = bool(rospy.get_param("~publish_debug", True))
        self.use_hog_fallback = bool(rospy.get_param("~use_hog_fallback", True))

        package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        model_dir = rospy.get_param("~model_dir", os.path.join(package_root, "models"))

        self.net = None
        self.hog = None
        local_paths = ensure_model_files(model_dir)
        prototxt = local_paths["deploy.prototxt"]
        model = local_paths["mobilenet_iter_73000.caffemodel"]

        if os.path.exists(prototxt) and os.path.exists(model):
            try:
                self.net = cv2.dnn.readNetFromCaffe(prototxt, model)
                rospy.loginfo("person_detector using MobileNet-SSD backend")
            except Exception as exc:
                rospy.logwarn("Cannot load MobileNet-SSD model: %s", exc)

        if self.net is None and self.use_hog_fallback:
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            rospy.logwarn("person_detector fallback to HOG detector (slower, no class scores)")

        if self.net is None and self.hog is None:
            rospy.logerr("No available person detector backend. Check model files or enable HOG fallback.")
            raise RuntimeError("person detector backend unavailable")

        self.person_pub = rospy.Publisher(self.person_topic, String, queue_size=10)
        self.debug_pub = rospy.Publisher(self.debug_topic, Image, queue_size=1)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)

        rospy.loginfo("person_detector started: image=%s person_topic=%s", self.image_topic, self.person_topic)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "CV bridge conversion failed: %s", exc)
            return

        h, w = frame.shape[:2]
        best = None

        if self.net is not None:
            blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)
            self.net.setInput(blob)
            detections = self.net.forward()

            for i in range(detections.shape[2]):
                score = float(detections[0, 0, i, 2])
                if score < self.score_threshold:
                    continue
                cls_idx = int(detections[0, 0, i, 1])
                if cls_idx >= len(CLASS_NAMES) or CLASS_NAMES[cls_idx] != "person":
                    continue

                box = detections[0, 0, i, 3:7] * [w, h, w, h]
                x1, y1, x2, y2 = box.astype(int)
                x1 = max(0, min(x1, w - 1))
                y1 = max(0, min(y1, h - 1))
                x2 = max(0, min(x2, w - 1))
                y2 = max(0, min(y2, h - 1))
                bw = max(1, x2 - x1)
                bh = max(1, y2 - y1)

                if best is None or score > best["score"]:
                    best = {
                        "score": score,
                        "bbox": [int(x1), int(y1), int(bw), int(bh)],
                        "cx": float((x1 + x2) / 2.0 / w),
                        "cy": float((y1 + y2) / 2.0 / h),
                    }
        else:
            rects, weights = self.hog.detectMultiScale(
                frame,
                winStride=(8, 8),
                padding=(8, 8),
                scale=1.05,
            )
            for (x, y, bw, bh), weight in zip(rects, weights):
                score = float(weight)
                x1, y1 = int(x), int(y)
                x2, y2 = int(x + bw), int(y + bh)
                if best is None or score > best["score"]:
                    best = {
                        "score": score,
                        "bbox": [int(x1), int(y1), int(bw), int(bh)],
                        "cx": float((x1 + x2) / 2.0 / w),
                        "cy": float((y1 + y2) / 2.0 / h),
                    }

        payload = {
            "stamp": msg.header.stamp.to_sec(),
            "frame_id": msg.header.frame_id,
            "image_width": int(w),
            "image_height": int(h),
            "found": best is not None,
            "label": "person",
        }
        if best is not None:
            bx, by, bw, bh = best["bbox"]
            payload.update(best)
            payload["bbox_height_ratio"] = float(bh) / float(h)
            payload["bbox_area_ratio"] = float(bw * bh) / float(w * h)

        self.person_pub.publish(String(data=json.dumps(payload, ensure_ascii=True)))

        if self.publish_debug:
            if best is not None:
                x, y, bw, bh = best["bbox"]
                cv2.rectangle(frame, (x, y), (x + bw, y + bh), (50, 220, 50), 2)
                cv2.putText(
                    frame,
                    "person {:.2f}".format(best["score"]),
                    (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (50, 220, 50),
                    2,
                )
            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
            except Exception:
                pass


if __name__ == "__main__":
    node = PersonDetectorNode()
    rospy.spin()
