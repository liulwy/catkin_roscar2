#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import select
import termios
import tty

import rospy
from geometry_msgs.msg import Twist


HELP_MSG = """
Reading from the keyboard and Publishing to /cmd_vel
---------------------------
Move:
   w
a  s  d
   x

w/x : forward/backward
a/d : turn left/right
s   : stop
1/2 : decrease/increase linear speed
3/4 : decrease/increase angular speed

q   : quit
"""


def get_key(keyboard_input, timeout=0.1):
     keyboard_fd = keyboard_input.fileno()
     tty.setraw(keyboard_fd)
     rlist, _, _ = select.select([keyboard_input], [], [], timeout)
     key = keyboard_input.read(1) if rlist else ""
     termios.tcsetattr(keyboard_fd, termios.TCSADRAIN, SETTINGS)
     return key


if __name__ == "__main__":
    keyboard_path = "/dev/tty"
    if not os.path.exists(keyboard_path):
        raise RuntimeError("/dev/tty is not available")

    keyboard_input = open(keyboard_path, "r")
    SETTINGS = termios.tcgetattr(keyboard_input.fileno())

    rospy.init_node("keyboard_teleop_node")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    linear = rospy.get_param("~linear", 0.2)
    angular = rospy.get_param("~angular", 0.8)
    linear_step = rospy.get_param("~linear_step", 0.02)
    angular_step = rospy.get_param("~angular_step", 0.05)

    twist = Twist()

    rospy.loginfo(HELP_MSG)
    rospy.loginfo("Initial linear=%.2f angular=%.2f", linear, angular)

    try:
        while not rospy.is_shutdown():
            key = get_key(keyboard_input)

            if key == "w":
                twist.linear.x = linear
                twist.angular.z = 0.0
            elif key == "x":
                twist.linear.x = -linear
                twist.angular.z = 0.0
            elif key == "a":
                twist.linear.x = 0.0
                twist.angular.z = angular
            elif key == "d":
                twist.linear.x = 0.0
                twist.angular.z = -angular
            elif key == "s":
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif key == "1":
                linear = max(0.0, linear - linear_step)
                rospy.loginfo("linear speed: %.2f", linear)
                continue
            elif key == "2":
                linear = linear + linear_step
                rospy.loginfo("linear speed: %.2f", linear)
                continue
            elif key == "3":
                angular = max(0.0, angular - angular_step)
                rospy.loginfo("angular speed: %.2f", angular)
                continue
            elif key == "4":
                angular = angular + angular_step
                rospy.loginfo("angular speed: %.2f", angular)
                continue
            elif key == "q":
                break
            else:
                continue

            pub.publish(twist)

    finally:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        termios.tcsetattr(keyboard_input.fileno(), termios.TCSADRAIN, SETTINGS)
        keyboard_input.close()
