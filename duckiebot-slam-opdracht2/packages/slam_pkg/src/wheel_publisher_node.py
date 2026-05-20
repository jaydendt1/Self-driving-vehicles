#!/usr/bin/env python3
"""
Wheel Publisher Node — Publiceert directe wielsnelheden via WASD-toetsen.
Topic: /<veh>/wheels_driver_node/wheels_cmd
"""
import os
import sys
import tty
import termios
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped

HELP = """
╔════════════════════════════════════════╗
║   Duckiebot Wheel Control (WASD)       ║
╠════════════════════════════════════════╣
║   W      vooruit                       ║
║   S      achteruit                     ║
║   A      links draaien                 ║
║   D      rechts draaien                ║
║   SPACE  stoppen                       ║
║   Q      afsluiten                     ║
╚════════════════════════════════════════╝
"""

SPEED = 0.4
TURN  = 0.3

BINDINGS = {
    'w': ( SPEED,  SPEED),
    's': (-SPEED, -SPEED),
    'a': (-TURN,   TURN),
    'd': ( TURN,  -TURN),
    ' ': (0.0,    0.0),
}


class WheelPublisherNode(DTROS):
    def __init__(self, node_name):
        super(WheelPublisherNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)
        self._veh = os.environ.get('VEHICLE_NAME', 'duckiebot')
        topic = f"/{self._veh}/wheels_driver_node/wheels_cmd"
        self._pub = rospy.Publisher(topic, WheelsCmdStamped, queue_size=1)
        rospy.loginfo(f"[{node_name}] Publisher op {topic}")

    def publish(self, left, right):
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = left
        msg.vel_right = right
        self._pub.publish(msg)

    def stop(self):
        self.publish(0.0, 0.0)

    def on_shutdown(self):
        self.stop()


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    node = WheelPublisherNode(node_name='wheel_publisher_node')
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    rate = rospy.Rate(10)
    left, right = 0.0, 0.0
    try:
        while not rospy.is_shutdown():
            key = get_key(settings)
            if key == 'q':
                rospy.loginfo("Afsluiten...")
                break
            if key in BINDINGS:
                left, right = BINDINGS[key]
                action = {'w':'↑ Vooruit','s':'↓ Achteruit',
                          'a':'← Links','d':'→ Rechts',' ':'■ Stop'}
                rospy.loginfo(f"{action.get(key, key)}  L={left:+.2f} R={right:+.2f}")
            node.publish(left, right)
            rate.sleep()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        rospy.loginfo("Gestopt.")


if __name__ == '__main__':
    main()
