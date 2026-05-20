#!/usr/bin/env python3
"""
Path Publisher Node
Verzamelt PoseStamped van /odometry/pose_stamped en publiceert als nav_msgs/Path
voor RViz.
"""
import os
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from duckietown.dtros import DTROS, NodeType


class PathPublisherNode(DTROS):
    def __init__(self, node_name):
        super(PathPublisherNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)

        self._veh = os.environ.get('VEHICLE_NAME', 'duckiebot')

        self.path = Path()
        self.path.header.frame_id = "odom"

        self.pub = rospy.Publisher(
            f"/{self._veh}/odometry/path", Path, queue_size=10, latch=True)

        rospy.Subscriber(
            f"/{self._veh}/odometry/pose_stamped",
            PoseStamped, self.cb_pose, queue_size=10)

        rospy.loginfo(f"[{node_name}] Path publisher klaar voor '{self._veh}'")

    def cb_pose(self, msg):
        msg.header.frame_id = "odom"
        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(msg)
        if len(self.path.poses) > 2000:
            self.path.poses = self.path.poses[-2000:]
        self.pub.publish(self.path)


if __name__ == '__main__':
    node = PathPublisherNode(node_name='path_publisher_node')
    rospy.spin()
