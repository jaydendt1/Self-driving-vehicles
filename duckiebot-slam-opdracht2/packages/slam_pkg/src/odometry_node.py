#!/usr/bin/env python3
"""
Odometry Node — Taak 1 (Opdracht 2)
Schat (x, y, θ) op basis van wheel-encoder ticks met een
differential-drive kinematisch model.

Publiceert:
  /<veh>/odometry/pose          (nav_msgs/Odometry)
  /<veh>/odometry/pose_stamped  (geometry_msgs/PoseStamped)
  TF odom -> base_link
"""
import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf_conversions


class OdometryNode(DTROS):
    def __init__(self, node_name):
        super(OdometryNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)

        self._veh = os.environ.get('VEHICLE_NAME', 'duckiebot')

        # ── Robot parameters (DB21J defaults) ──
        # Pas aan voor jouw bot indien gekalibreerd
        self.wheel_radius   = rospy.get_param("~wheel_radius",   0.0318)
        self.wheel_baseline = rospy.get_param("~wheel_baseline", 0.1)
        self.ticks_per_rev  = rospy.get_param("~ticks_per_rev",  135)

        # ── Pose state ──
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Encoder state
        self.prev_l = None
        self.prev_r = None
        self.res_l  = None
        self.res_r  = None

        # TF
        self.tf_br = tf2_ros.TransformBroadcaster()

        # Publishers
        self.pub_odom = rospy.Publisher(
            f"/{self._veh}/odometry/pose", Odometry, queue_size=10)
        self.pub_pose = rospy.Publisher(
            f"/{self._veh}/odometry/pose_stamped", PoseStamped, queue_size=10)

        # Subscribers — beide naam-varianten
        for topic in [
            f"/{self._veh}/left_wheel_encoder_node/tick",
            f"/{self._veh}/left_wheel_encoder_driver_node/tick",
        ]:
            rospy.Subscriber(topic, WheelEncoderStamped, self.cb_left, queue_size=10)
        for topic in [
            f"/{self._veh}/right_wheel_encoder_node/tick",
            f"/{self._veh}/right_wheel_encoder_driver_node/tick",
        ]:
            rospy.Subscriber(topic, WheelEncoderStamped, self.cb_right, queue_size=10)

        rospy.loginfo(f"[{node_name}] Odometry klaar voor '{self._veh}'")
        rospy.loginfo(f"  wheel_radius={self.wheel_radius} baseline={self.wheel_baseline} "
                      f"ticks/rev={self.ticks_per_rev}")
        rospy.loginfo(f"  Publiceert -> /{self._veh}/odometry/pose")

    def cb_left(self, msg):
        if self.res_l is None and msg.resolution > 0:
            self.res_l = msg.resolution
            rospy.loginfo(f"Gebruik linker encoder resolutie {self.res_l} ticks/rev")
        if self.prev_l is not None:
            self._update(msg.data - self.prev_l, 0)
        self.prev_l = msg.data

    def cb_right(self, msg):
        if self.res_r is None and msg.resolution > 0:
            self.res_r = msg.resolution
            rospy.loginfo(f"Gebruik rechter encoder resolutie {self.res_r} ticks/rev")
        if self.prev_r is not None:
            self._update(0, msg.data - self.prev_r)
        self.prev_r = msg.data

    def _update(self, dl, dr):
        """Differential-drive kinematisch model."""
        res_l = self.res_l or self.ticks_per_rev
        res_r = self.res_r or self.ticks_per_rev
        dist_l = (dl / res_l) * 2.0 * math.pi * self.wheel_radius
        dist_r = (dr / res_r) * 2.0 * math.pi * self.wheel_radius

        ds     = (dist_l + dist_r) / 2.0
        dtheta = (dist_r - dist_l) / self.wheel_baseline

        self.theta += dtheta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.x += ds * math.cos(self.theta)
        self.y += ds * math.sin(self.theta)

        rospy.loginfo_throttle(1,
            f"[odometry] x={self.x:+.3f}m  y={self.y:+.3f}m  "
            f"θ={math.degrees(self.theta):+.1f}°")

        self._publish()

    def _publish(self):
        now = rospy.Time.now()
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.theta)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        self.pub_odom.publish(odom)

        # PoseStamped
        ps = PoseStamped()
        ps.header = odom.header
        ps.pose = odom.pose.pose
        self.pub_pose.publish(ps)

        # TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_br.sendTransform(t)


if __name__ == '__main__':
    node = OdometryNode(node_name='odometry_node')
    rospy.spin()
