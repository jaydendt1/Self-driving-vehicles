#!/usr/bin/env python3
"""
Sensorfusie Node (EKF) — Taak 3 (Opdracht 2)

State:  [x, y, θ]
Predict: bij elke odometry-sample (delta x, y, θ)
Update:  bij elke visuele pixel-flow meting van slam_node
         (omgezet naar geschatte body-frame snelheid)

Publiceert:
  /<veh>/fusion_node/pose          (nav_msgs/Odometry, incl. covariance)
  /<veh>/fusion_node/pose_stamped  (geometry_msgs/PoseStamped)
  /<veh>/fusion_node/path          (nav_msgs/Path, voor RViz)
"""
import os
import math
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import tf_conversions


class FusionNode(DTROS):
    def __init__(self, node_name):
        super(FusionNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)

        self._veh = os.environ.get('VEHICLE_NAME', 'duckiebot')

        # ── EKF state ──
        self.mu = np.zeros(3)
        self.sigma = np.eye(3) * 0.01

        # Process noise — odometry vertrouwen
        self.Q = np.diag([0.005, 0.005, 0.002])
        # Measurement noise — visie vertrouwen (groot = minder vertrouwen)
        self.R = np.diag([2.0, 2.0])    # Hoge R = visie zwak gewicht, vertrouw vooral odometry

        # Vorige odometry
        self.prev_x = None
        self.prev_y = None
        self.prev_theta = None
        self.prev_t = None

        # Laatste body-frame snelheid (voor visuele update)
        self.last_v_body = np.zeros(2)

        # Path voor RViz
        self.path = Path()
        self.path.header.frame_id = "map"

        # Publishers
        self.pub_pose = rospy.Publisher(
            f"/{self._veh}/fusion_node/pose", Odometry, queue_size=10)
        self.pub_pose_stamped = rospy.Publisher(
            f"/{self._veh}/fusion_node/pose_stamped", PoseStamped, queue_size=10)
        self.pub_path = rospy.Publisher(
            f"/{self._veh}/fusion_node/path", Path, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber(f"/{self._veh}/odometry/pose",
                         Odometry, self.cb_odom)
        rospy.Subscriber(f"/{self._veh}/slam_node/correction",
                         Float32MultiArray, self.cb_visual)

        rospy.loginfo(f"[{node_name}] EKF Fusion klaar voor '{self._veh}'")

    # ── Predict (odometry) ────────────────────────────────────────────

    def cb_odom(self, msg):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        ot = math.atan2(siny, cosy)
        now = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()

        if self.prev_x is None:
            self.prev_x, self.prev_y, self.prev_theta = ox, oy, ot
            self.prev_t = now
            return

        dx = ox - self.prev_x
        dy = oy - self.prev_y
        dtheta = math.atan2(math.sin(ot - self.prev_theta),
                            math.cos(ot - self.prev_theta))
        dt = max(1e-3, now - self.prev_t)

        self.prev_x, self.prev_y, self.prev_theta = ox, oy, ot
        self.prev_t = now

        # Predict
        self.mu[0] += dx
        self.mu[1] += dy
        self.mu[2] += dtheta
        self.mu[2] = math.atan2(math.sin(self.mu[2]), math.cos(self.mu[2]))

        # Body-frame snelheid (voor update step)
        theta = self.mu[2]
        c, s = math.cos(-theta), math.sin(-theta)
        self.last_v_body = np.array([
            (c * dx - s * dy) / dt,
            (s * dx + c * dy) / dt,
        ])

        F = np.eye(3)
        self.sigma = F @ self.sigma @ F.T + self.Q

        rospy.loginfo_throttle(2,
            f"[EKF predict] x={self.mu[0]:+.3f} y={self.mu[1]:+.3f} "
            f"θ={math.degrees(self.mu[2]):+.1f}°")

        self._publish()

    # ── Update (visuele meting) ───────────────────────────────────────

    def cb_visual(self, msg):
        if len(msg.data) < 2:
            return
        flow_dx = float(msg.data[0])
        flow_dy = float(msg.data[1])

        fps = 30.0
        pixel_to_m = 0.0015

        # Body-frame snelheid uit visie
        v_meas_body = np.array([
            flow_dy * pixel_to_m * fps,    # vooruit (+x body)
            -flow_dx * pixel_to_m * fps,   # links (+y body)
        ])

        # Naar wereldverplaatsing over één frame
        dt = 1.0 / fps
        theta = self.mu[2]
        c, s = math.cos(theta), math.sin(theta)
        z_world = np.array([
            (c * v_meas_body[0] - s * v_meas_body[1]) * dt,
            (s * v_meas_body[0] + c * v_meas_body[1]) * dt,
        ])
        h_world = np.array([
            (c * self.last_v_body[0] - s * self.last_v_body[1]) * dt,
            (s * self.last_v_body[0] + c * self.last_v_body[1]) * dt,
        ])
        innovation = z_world - h_world

        H = np.array([[1, 0, 0],
                      [0, 1, 0]])
        S = H @ self.sigma @ H.T + self.R
        try:
            K = self.sigma @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        self.mu = self.mu + K @ innovation
        self.mu[2] = math.atan2(math.sin(self.mu[2]), math.cos(self.mu[2]))
        self.sigma = (np.eye(3) - K @ H) @ self.sigma

        rospy.loginfo_throttle(3,
            f"[EKF update] innov=({innovation[0]:+.4f}, {innovation[1]:+.4f})")
        self._publish()

    # ── Publishing ────────────────────────────────────────────────────

    def _publish(self):
        now = rospy.Time.now()
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.mu[2])

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.mu[0]
        odom.pose.pose.position.y = self.mu[1]
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        cov = [0.0] * 36
        cov[0]  = self.sigma[0, 0]
        cov[1]  = self.sigma[0, 1]
        cov[5]  = self.sigma[0, 2]
        cov[6]  = self.sigma[1, 0]
        cov[7]  = self.sigma[1, 1]
        cov[11] = self.sigma[1, 2]
        cov[30] = self.sigma[2, 0]
        cov[31] = self.sigma[2, 1]
        cov[35] = self.sigma[2, 2]
        odom.pose.covariance = cov

        self.pub_pose.publish(odom)

        ps = PoseStamped()
        ps.header = odom.header
        ps.pose = odom.pose.pose
        self.pub_pose_stamped.publish(ps)

        self.path.header.stamp = now
        self.path.poses.append(ps)
        if len(self.path.poses) > 2000:
            self.path.poses = self.path.poses[-2000:]
        self.pub_path.publish(self.path)


if __name__ == '__main__':
    node = FusionNode(node_name='fusion_node')
    rospy.spin()
