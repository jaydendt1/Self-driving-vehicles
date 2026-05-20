#!/usr/bin/env python3
"""
Vision SLAM Node — Taak 2 (Opdracht 2)
Monocular vision SLAM: ORB feature detection + Lucas-Kanade optical flow.
Bouwt een 2D top-down kaart van landmarks en publiceert een visuele
correctie (pixel-flow) voor de fusion node.

Abonneert op compressed EN raw camera topics als fallback — handig
als jouw camera node alleen één van beide publiceert.

Publiceert:
  /<veh>/slam_node/features/compressed   (sensor_msgs/CompressedImage)
  /<veh>/slam_node/map/compressed         (sensor_msgs/CompressedImage)
  /<veh>/slam_node/correction             (std_msgs/Float32MultiArray)
  /<veh>/slam_node/landmarks              (geometry_msgs/PoseArray)
"""
import os
import math
import rospy
import cv2
import numpy as np
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge


class SLAMNode(DTROS):
    def __init__(self, node_name):
        super(SLAMNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)

        self._veh = os.environ.get('VEHICLE_NAME', 'duckiebot')
        self._bridge = CvBridge()

        # Camera intrinsics (DB21 default — vervang met calibratie indien beschikbaar)
        self.fx = 305.0
        self.fy = 305.0
        self.cx = 320.0
        self.cy = 240.0
        self.camera_height = 0.10   # m boven grond

        # ORB
        self.orb = cv2.ORB_create(500)
        self.bf  = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Tracking state
        self.prev_gray = None
        self.prev_descriptors = None
        self.prev_pts = None

        # Map state
        self.map_points = []
        self.trajectory = [(0.0, 0.0)]

        # Pose (van fusion of fallback van odometry)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._got_fusion_pose = False

        # Visuele bewegingsschatting
        self.visual_dx = 0.0
        self.visual_dy = 0.0
        self.frame_count = 0
        self._first_frame_log_done = False

        # Map canvas
        self.map_size = 600
        self.scale = 150
        self.map_img = np.zeros((self.map_size, self.map_size, 3), dtype=np.uint8)

        # Publishers
        self.pub_features = rospy.Publisher(
            f"/{self._veh}/slam_node/features/compressed", CompressedImage, queue_size=1)
        self.pub_map = rospy.Publisher(
            f"/{self._veh}/slam_node/map/compressed", CompressedImage, queue_size=1)
        self.pub_correction = rospy.Publisher(
            f"/{self._veh}/slam_node/correction", Float32MultiArray, queue_size=1)
        self.pub_landmarks = rospy.Publisher(
            f"/{self._veh}/slam_node/landmarks", PoseArray, queue_size=1)

        # Camera subscribers — beide formats
        rospy.Subscriber(f"/{self._veh}/camera_node/image/compressed",
                         CompressedImage, self.cb_compressed,
                         queue_size=1, buff_size=2**24)
        rospy.Subscriber(f"/{self._veh}/camera_node/image/raw",
                         Image, self.cb_raw,
                         queue_size=1, buff_size=2**24)

        # Pose subscribers (fusion heeft voorrang)
        rospy.Subscriber(f"/{self._veh}/fusion_node/pose",
                         Odometry, self.cb_fusion_pose)
        rospy.Subscriber(f"/{self._veh}/odometry/pose",
                         Odometry, self.cb_odom_pose)

        # Waarschuwing als geen camera binnenkomt
        rospy.Timer(rospy.Duration(5.0), self._check_camera_alive)

        rospy.loginfo(f"[{node_name}] SLAM klaar voor '{self._veh}'")
        rospy.loginfo(f"  Wacht op camera op /{self._veh}/camera_node/image/...")

    def _check_camera_alive(self, _evt):
        if self.frame_count == 0:
            rospy.logwarn("⚠ SLAM: nog GEEN cameraframes ontvangen na 5s!")
            rospy.logwarn("  → run `dts devel run -H ROBOT -L diagnostic` voor details")

    # ── Pose callbacks ──────────────────────────────────────────────────

    def cb_fusion_pose(self, msg):
        self._got_fusion_pose = True
        self._set_pose(msg)

    def cb_odom_pose(self, msg):
        if not self._got_fusion_pose:
            self._set_pose(msg)

    def _set_pose(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny, cosy)
        if self.trajectory:
            lx, ly = self.trajectory[-1]
            if abs(self.x - lx) > 0.01 or abs(self.y - ly) > 0.01:
                self.trajectory.append((self.x, self.y))
                if len(self.trajectory) > 5000:
                    self.trajectory = self.trajectory[-5000:]

    # ── Camera callbacks ────────────────────────────────────────────────

    def cb_compressed(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                rospy.logwarn_throttle(5, "[SLAM] imdecode gaf None terug")
                return
            self._process(frame, "compressed")
        except Exception as e:
            rospy.logerr_throttle(5, f"[SLAM] compressed decode error: {e}")

    def cb_raw(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._process(frame, "raw")
        except Exception as e:
            rospy.logerr_throttle(5, f"[SLAM] raw decode error: {e}")

    def _process(self, frame, source):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.frame_count += 1
        if not self._first_frame_log_done:
            h, w = frame.shape[:2]
            rospy.loginfo(f"[SLAM] Eerste frame ontvangen: {w}x{h} ({source})")
            self._first_frame_log_done = True

        # ORB detectie
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)

        # Lucas-Kanade tracking
        flow_viz = frame.copy()
        if self.prev_gray is not None and self.prev_pts is not None and len(self.prev_pts) > 0:
            lk = dict(winSize=(15, 15), maxLevel=2,
                      criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
            curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                self.prev_gray, gray, self.prev_pts, None, **lk)

            if curr_pts is not None and status is not None:
                # Reshape naar 2D (N,2) — calcOpticalFlowPyrLK levert (N,1,2)
                mask = status.flatten() == 1
                good_new = curr_pts.reshape(-1, 2)[mask]
                good_old = self.prev_pts.reshape(-1, 2)[mask]

                for new, old in zip(good_new, good_old):
                    a, b = int(round(float(new[0]))), int(round(float(new[1])))
                    c, d = int(round(float(old[0]))), int(round(float(old[1])))
                    cv2.arrowedLine(flow_viz, (c, d), (a, b),
                                    (0, 255, 255), 1, tipLength=0.3)
                    cv2.circle(flow_viz, (a, b), 2, (0, 200, 0), -1)

                if len(good_new) > 5:
                    flow = good_new - good_old
                    self.visual_dx = float(np.median(flow[:, 0]).item())
                    self.visual_dy = float(np.median(flow[:, 1]).item())

                    corr = Float32MultiArray()
                    corr.data = [self.visual_dx, self.visual_dy]
                    self.pub_correction.publish(corr)

                self.prev_pts = good_new.reshape(-1, 1, 2) if len(good_new) > 10 else None
            else:
                self.prev_pts = None

        if self.prev_pts is None and keypoints:
            self.prev_pts = np.array(
                [[kp.pt] for kp in keypoints[:100]], dtype=np.float32)

        # ORB matching → landmarks
        if (self.prev_descriptors is not None and
                descriptors is not None and len(descriptors) > 10):
            matches = sorted(self.bf.match(self.prev_descriptors, descriptors),
                             key=lambda m: m.distance)
            good = [m for m in matches[:40] if m.distance < 50]
            for m in good:
                pt = keypoints[m.trainIdx].pt
                world = self._pixel_to_world(float(pt[0]), float(pt[1]))
                if world is None:
                    continue
                strength = 1.0 - m.distance / 100.0
                self.map_points.append((world[0], world[1], max(0.1, strength)))
            if len(self.map_points) > 3000:
                self.map_points = self.map_points[-3000:]

        self.prev_gray = gray.copy()
        self.prev_descriptors = descriptors

        # Visualisatie
        for kp in keypoints:
            x, y = int(float(kp.pt[0])), int(float(kp.pt[1]))
            cv2.drawMarker(flow_viz, (x, y), (0, 255, 0), cv2.MARKER_CROSS, 6, 1)
        cv2.line(flow_viz, (0, int(self.cy)),
                 (flow_viz.shape[1], int(self.cy)), (100, 100, 100), 1)

        # HUD
        cv2.rectangle(flow_viz, (0, 0), (320, 85), (0, 0, 0), -1)
        cv2.putText(flow_viz, f"ORB features : {len(keypoints)}",
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        cv2.putText(flow_viz, f"Map landmarks: {len(self.map_points)}",
                    (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 150, 0), 1)
        cv2.putText(flow_viz, f"Flow dx={self.visual_dx:+.1f} dy={self.visual_dy:+.1f}",
                    (8, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1)
        cv2.putText(flow_viz, f"Frame: {self.frame_count}",
                    (8, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

        # Publish features
        feat = self._bridge.cv2_to_compressed_imgmsg(flow_viz)
        feat.header.stamp = rospy.Time.now()
        self.pub_features.publish(feat)

        # Publish map + landmarks
        self._redraw_map()
        self._publish_map()
        self._publish_landmarks()

    # ── Pixel naar wereld ──────────────────────────────────────────────

    def _pixel_to_world(self, px, py):
        u = px - self.cx
        v = py - self.cy
        if v <= 1:
            return None
        depth = self.camera_height * self.fy / v
        if depth <= 0 or depth > 5.0:
            return None
        lateral = u * depth / self.fx
        wx = self.x + depth * math.cos(self.theta) - lateral * math.sin(self.theta)
        wy = self.y + depth * math.sin(self.theta) + lateral * math.cos(self.theta)
        return wx, wy

    # ── Map rendering ──────────────────────────────────────────────────

    def _world_to_px(self, wx, wy):
        c = self.map_size // 2
        return (int(c + wx * self.scale), int(c - wy * self.scale))

    def _redraw_map(self):
        self.map_img[:] = (15, 15, 15)
        c = self.map_size // 2
        for d in range(-4, 5):
            px = c + int(d * self.scale)
            py = c + int(d * self.scale)
            if 0 <= px < self.map_size:
                cv2.line(self.map_img, (px, 0), (px, self.map_size), (40, 40, 40), 1)
            if 0 <= py < self.map_size:
                cv2.line(self.map_img, (0, py), (self.map_size, py), (40, 40, 40), 1)

        # Landmarks
        for (mx, my, s) in self.map_points[-800:]:
            pt = self._world_to_px(mx, my)
            if 0 <= pt[0] < self.map_size and 0 <= pt[1] < self.map_size:
                r = int(255 * (1 - s))
                b = int(255 * s)
                cv2.circle(self.map_img, pt, 1, (b, 80, r), -1)

        # Trajectory
        for i in range(1, len(self.trajectory)):
            p1 = self._world_to_px(*self.trajectory[i - 1])
            p2 = self._world_to_px(*self.trajectory[i])
            if all(0 <= v < self.map_size for v in [p1[0], p1[1], p2[0], p2[1]]):
                cv2.line(self.map_img, p1, p2, (0, 200, 0), 2)

        # Start
        s = self._world_to_px(0.0, 0.0)
        cv2.drawMarker(self.map_img, s, (0, 255, 255), cv2.MARKER_STAR, 16, 2)
        cv2.putText(self.map_img, "START",
                    (s[0] + 8, s[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)

        # Pose
        cur = self._world_to_px(self.x, self.y)
        if 0 <= cur[0] < self.map_size and 0 <= cur[1] < self.map_size:
            cv2.circle(self.map_img, cur, 8, (0, 0, 255), -1)
            end = (int(cur[0] + 20 * math.cos(self.theta)),
                   int(cur[1] - 20 * math.sin(self.theta)))
            cv2.arrowedLine(self.map_img, cur, end, (0, 180, 255), 2, tipLength=0.35)

        # HUD
        cv2.rectangle(self.map_img, (0, 0), (240, 70), (0, 0, 0), -1)
        cv2.putText(self.map_img, f"x = {self.x:+.3f} m",
                    (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(self.map_img, f"y = {self.y:+.3f} m",
                    (8, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(self.map_img, f"θ = {math.degrees(self.theta):+.1f}°  pts={len(self.map_points)}",
                    (8, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 255), 1)

    def _publish_map(self):
        try:
            msg = self._bridge.cv2_to_compressed_imgmsg(self.map_img)
            msg.header.stamp = rospy.Time.now()
            self.pub_map.publish(msg)
        except Exception as e:
            rospy.logerr_throttle(5, f"[SLAM] map publish error: {e}")

    def _publish_landmarks(self):
        try:
            pa = PoseArray()
            pa.header.stamp = rospy.Time.now()
            pa.header.frame_id = "map"
            for (mx, my, _) in self.map_points[-200:]:
                p = Pose()
                p.position.x = mx
                p.position.y = my
                p.orientation.w = 1.0
                pa.poses.append(p)
            self.pub_landmarks.publish(pa)
        except Exception as e:
            rospy.logerr_throttle(5, f"[SLAM] landmark publish error: {e}")


if __name__ == '__main__':
    node = SLAMNode(node_name='slam_node')
    rospy.spin()
