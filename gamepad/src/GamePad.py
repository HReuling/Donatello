#!/usr/bin/env python3

import rospy
import os
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


def read_pad(data):
    twist.linear.x =0.5*(1 - data.axes[4] - (1-data.axes[5]))
    twist.angular.z = 4* data.axes[0]


def remote_control():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rospy.Subscriber('joy', Joy, read_pad)

    rospy.init_node('Joy2Cmd_vel')

    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish(twist)
        r.sleep()

if __name__ == '__main__':
    rospy.set_param('joy_node/dev', '/dev/input/js4')
    os.system('gnome-terminal --tab -- /bin/bash -c \
                  "rosrun joy joy_node;\
                  exec bash"')
    twist = Twist()
    remote_control()
