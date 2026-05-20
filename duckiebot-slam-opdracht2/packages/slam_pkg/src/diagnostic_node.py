#!/usr/bin/env python3
"""
Diagnostic Node
Controleert alle topics die nodig zijn voor Opdracht 2:
  - Camera
  - Wheel encoders (links + rechts)
  - Wheel commands
Geeft duidelijke meldingen welke topics WEL en NIET binnenkomen,
zodat je camera-issues snel kan diagnosticeren.
"""
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import WheelEncoderStamped


class DiagnosticNode(DTROS):
    def __init__(self, node_name):
        super(DiagnosticNode, self).__init__(
            node_name=node_name, node_type=NodeType.DIAGNOSTICS)

        veh = os.environ.get('VEHICLE_NAME', 'duckiebot')
        self._veh = veh

        # Counter voor elke topic
        self.counts = {
            'camera_compressed':         0,
            'camera_raw':                0,
            'left_encoder':              0,
            'right_encoder':             0,
            'left_encoder_driver':       0,
            'right_encoder_driver':      0,
        }

        # Subscribers — meerdere camera-topics tegelijk
        topics = [
            (f"/{veh}/camera_node/image/compressed",        CompressedImage, 'camera_compressed'),
            (f"/{veh}/camera_node/image/raw",               Image,           'camera_raw'),
            (f"/{veh}/left_wheel_encoder_node/tick",        WheelEncoderStamped, 'left_encoder'),
            (f"/{veh}/right_wheel_encoder_node/tick",       WheelEncoderStamped, 'right_encoder'),
            (f"/{veh}/left_wheel_encoder_driver_node/tick", WheelEncoderStamped, 'left_encoder_driver'),
            (f"/{veh}/right_wheel_encoder_driver_node/tick",WheelEncoderStamped, 'right_encoder_driver'),
        ]

        rospy.loginfo("=" * 60)
        rospy.loginfo(f"[diagnostic] Subscribed to topics for '{veh}':")
        for topic, _, _ in topics:
            rospy.loginfo(f"  • {topic}")
        rospy.loginfo("=" * 60)

        for topic, msg_type, key in topics:
            rospy.Subscriber(topic, msg_type,
                             lambda msg, k=key: self._tick(k),
                             queue_size=1, buff_size=2**24)

        # Print samenvatting elke 3 seconden
        rospy.Timer(rospy.Duration(3.0), self._report)

    def _tick(self, key):
        self.counts[key] += 1

    def _report(self, _evt):
        rospy.loginfo("─── Topic activiteit (laatste 3s, totalen) ───")
        labels = {
            'camera_compressed':    'Camera (compressed)',
            'camera_raw':           'Camera (raw)       ',
            'left_encoder':         'Encoder L          ',
            'right_encoder':        'Encoder R          ',
            'left_encoder_driver':  'Encoder L (driver) ',
            'right_encoder_driver': 'Encoder R (driver) ',
        }
        for key, label in labels.items():
            n = self.counts[key]
            mark = "✓" if n > 0 else "✗"
            rospy.loginfo(f"  {mark} {label}: {n} berichten")

        # Specifieke camera-melding
        if self.counts['camera_compressed'] == 0 and self.counts['camera_raw'] == 0:
            rospy.logwarn("⚠ GEEN camera data ontvangen!")
            rospy.logwarn("  Mogelijke oorzaken:")
            rospy.logwarn("  1. Camera node draait niet — check: dts duckiebot diagnose")
            rospy.logwarn(f"  2. Verkeerde VEHICLE_NAME (nu: {self._veh})")
            rospy.logwarn("  3. ROS_MASTER_URI niet correct ingesteld")
            rospy.logwarn("  4. Camera hardware defect")
            rospy.logwarn("  → Probeer eens: rostopic list | grep camera")

        if self.counts['left_encoder'] + self.counts['left_encoder_driver'] == 0:
            rospy.logwarn("⚠ Geen LINKER encoder data")
        if self.counts['right_encoder'] + self.counts['right_encoder_driver'] == 0:
            rospy.logwarn("⚠ Geen RECHTER encoder data")


if __name__ == '__main__':
    node = DiagnosticNode(node_name='diagnostic_node')
    rospy.spin()
