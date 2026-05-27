#!/usr/bin/env python3
"""
Argo Mini keyboard teleop — publishes Twist to /cmd_vel.

  w   forward          i   forward-left arc
  s   reverse          o   forward-right arc
  a   turn left        ,   reverse-left arc
  d   turn right       .   reverse-right arc
  space / k   stop
  Ctrl+C      quit
"""

import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

LIN = 0.15
ANG = 0.50

BINDINGS = {
    'w': ( LIN,  0.0,  'FORWARD'),
    's': (-LIN,  0.0,  'REVERSE'),
    'a': ( 0.0,  ANG,  'LEFT'),
    'd': ( 0.0, -ANG,  'RIGHT'),
    'i': ( LIN,  ANG,  'FWD-LEFT'),
    'o': ( LIN, -ANG,  'FWD-RIGHT'),
    ',': (-LIN,  ANG,  'REV-LEFT'),
    '.': (-LIN, -ANG,  'REV-RIGHT'),
    ' ': ( 0.0,  0.0,  'STOP'),
    'k': ( 0.0,  0.0,  'STOP'),
}

BANNER = """
======================================
  ARGO MINI TELEOP  (/cmd_vel)
======================================
  w        forward
  s        reverse
  a/d      turn left / right
  i/o      forward arc left / right
  ,/.      reverse arc left / right
  space    stop
  Ctrl+C   quit
======================================
"""

def get_key(old_settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return key

def main():
    rclpy.init()
    node = Node('simple_teleop')
    pub  = node.create_publisher(Twist, '/cmd_vel', 10)
    settings = termios.tcgetattr(sys.stdin)
    print(BANNER)
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == '\x03':
                break
            if key not in BINDINGS:
                continue
            lin, ang, label = BINDINGS[key]
            msg = Twist()
            msg.linear.x  = lin
            msg.angular.z = ang
            pub.publish(msg)
            print(f'\r  {label:<12}  lin={lin:+.2f}  ang={ang:+.2f}   ', end='', flush=True)
    finally:
        pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print('\n[teleop] stopped.')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
