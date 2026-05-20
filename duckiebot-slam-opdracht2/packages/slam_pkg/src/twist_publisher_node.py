#!/usr/bin/env python3
"""
Twist Publisher Node — Chassis-level (v, omega) commando demo.
Publiceert: /<veh>/car_cmd_switch_node/cmd
"""
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped


class TwistPublisherNode(DTROS):
    def __init__(self, node_name):
        super(TwistPublisherNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)
        self._veh = os.environ.get('VEHICLE_NAME', 'duckiebot')
        topic = f"/{self._veh}/car_cmd_switch_node/cmd"
        self._pub = rospy.Publisher(topic, Twist2DStamped, queue_size=1)
        rospy.loginfo(f"[{node_name}] Publisher op {topic}")

    def send_cmd(self, v, omega, duration):
        msg = Twist2DStamped(v=v, omega=omega)
        msg.header.stamp = rospy.Time.now()
        end = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(10)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            self._pub.publish(msg)
            rate.sleep()
        self.stop()

    def stop(self):
        msg = Twist2DStamped(v=0.0, omega=0.0)
        msg.header.stamp = rospy.Time.now()
        self._pub.publish(msg)

    def on_shutdown(self):
        self.stop()


if __name__ == '__main__':
    node = TwistPublisherNode(node_name='twist_publisher_node')
    rospy.sleep(1.0)
    rospy.loginfo("Demo: 0.2 m/s vooruit voor 2s...")
    node.send_cmd(0.2, 0.0, 2.0)
    rospy.loginfo("Demo: draaien voor 2s...")
    node.send_cmd(0.0, 1.0, 2.0)
    rospy.loginfo("Klaar.")
    rospy.spin()
