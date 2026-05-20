#!/usr/bin/env python3
"""
Camera Reader Node — Basis subscriber op de camera.
Abonneert op beide compressed EN raw topics, gebruikt wat binnenkomt.
Print resolutie en framerate; nuttig om te checken of de camera werkt.
"""
import os
import time
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class CameraReaderNode(DTROS):
    def __init__(self, node_name):
        super(CameraReaderNode, self).__init__(
            node_name=node_name, node_type=NodeType.VISUALIZATION)

        self._veh = os.environ.get('VEHICLE_NAME', 'duckiebot')
        self._bridge = CvBridge()
        self._last_log = time.time()
        self._count_compressed = 0
        self._count_raw = 0

        # Beide topic-varianten
        topic_compressed = f"/{self._veh}/camera_node/image/compressed"
        topic_raw = f"/{self._veh}/camera_node/image/raw"

        rospy.Subscriber(topic_compressed, CompressedImage,
                         self.cb_compressed, queue_size=1, buff_size=2**24)
        rospy.Subscriber(topic_raw, Image,
                         self.cb_raw, queue_size=1, buff_size=2**24)

        rospy.loginfo(f"[{node_name}] Listening on:")
        rospy.loginfo(f"  • {topic_compressed}")
        rospy.loginfo(f"  • {topic_raw}")
        rospy.loginfo("Wachten op camera data...")

    def cb_compressed(self, msg):
        try:
            img = self._bridge.compressed_imgmsg_to_cv2(msg)
            self._count_compressed += 1
            self._report(img, "compressed")
        except Exception as e:
            rospy.logerr_throttle(5, f"Compressed image error: {e}")

    def cb_raw(self, msg):
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._count_raw += 1
            self._report(img, "raw")
        except Exception as e:
            rospy.logerr_throttle(5, f"Raw image error: {e}")

    def _report(self, img, kind):
        now = time.time()
        if now - self._last_log >= 2.0:
            h, w = img.shape[:2]
            total = self._count_compressed + self._count_raw
            rospy.loginfo(
                f"[camera] {kind} {w}x{h} | total frames: "
                f"compressed={self._count_compressed} raw={self._count_raw} "
                f"≈{total/(now - self._last_log + 1e-6):.1f} fps")
            self._last_log = now


if __name__ == '__main__':
    node = CameraReaderNode(node_name='camera_reader_node')
    rospy.spin()
