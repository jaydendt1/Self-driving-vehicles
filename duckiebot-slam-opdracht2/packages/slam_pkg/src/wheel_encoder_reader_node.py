#!/usr/bin/env python3
"""
Wheel Encoder Reader Node — Basis subscriber op de wielencoders.
"""
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped


class WheelEncoderReaderNode(DTROS):
    def __init__(self, node_name):
        super(WheelEncoderReaderNode, self).__init__(
            node_name=node_name, node_type=NodeType.PERCEPTION)

        self._veh = os.environ.get('VEHICLE_NAME', 'duckiebot')

        self._ticks_l = None
        self._ticks_r = None
        self._res_l = None
        self._res_r = None

        # Beide naam-varianten
        topics = [
            (f"/{self._veh}/left_wheel_encoder_node/tick",         self.cb_left),
            (f"/{self._veh}/left_wheel_encoder_driver_node/tick",  self.cb_left),
            (f"/{self._veh}/right_wheel_encoder_node/tick",        self.cb_right),
            (f"/{self._veh}/right_wheel_encoder_driver_node/tick", self.cb_right),
        ]
        for t, cb in topics:
            rospy.Subscriber(t, WheelEncoderStamped, cb, queue_size=10)
            rospy.loginfo(f"[{node_name}] Subscribed to {t}")

    def cb_left(self, msg):
        if self._res_l is None:
            self._res_l = msg.resolution
            rospy.loginfo(f"Linker encoder resolutie: {msg.resolution} ticks/rev")
        self._ticks_l = msg.data
        self._log()

    def cb_right(self, msg):
        if self._res_r is None:
            self._res_r = msg.resolution
            rospy.loginfo(f"Rechter encoder resolutie: {msg.resolution} ticks/rev")
        self._ticks_r = msg.data
        self._log()

    def _log(self):
        if self._ticks_l is not None and self._ticks_r is not None:
            rospy.loginfo_throttle(1,
                f"[encoders] L={self._ticks_l} ticks  R={self._ticks_r} ticks  "
                f"Δ={self._ticks_l - self._ticks_r}")


if __name__ == '__main__':
    node = WheelEncoderReaderNode(node_name='wheel_encoder_reader_node')
    rospy.spin()
