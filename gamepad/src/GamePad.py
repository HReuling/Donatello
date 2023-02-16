#!/usr/bin/env python3
#Test
import rospy
import os
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

js_num = 0;
js_mode = 0; #0 fuer Xbox Controller, 1 fuer Logitech


def read_pad(data):
    if (js_mode == 0):
        twist.linear.x =0.5*(1 - data.axes[5] - (1-data.axes[2]))
        twist.angular.z = 3* data.axes[0]
    if (js_mode == 1):
        twist.linear.x = data.axes[3]
        twist.angular.z = 3* data.axes[0]


def remote_control():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rospy.Subscriber('joy', Joy, read_pad)

    #rospy.init_node('Joy2Cmd_vel')

    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish(twist)
        r.sleep()

if __name__ == '__main__':

    # Nummer und Mode des JoyPads aus den Argumenten auslesen
    # Mode 0: Xbox
    # Mode 1: Logitech
    rospy.init_node("Joy2Cmd_vel")
    js_mode = rospy.get_param("~js_mode")

    rospy.loginfo("Js Mode:" + str(js_mode))

    # myargv = rospy.myargv(argv=sys.argv)
    # if len(myargv) > 1:
    #     js_num = myargv[1]
    # if len(myargv) > 2:
    #     js_mode = myargv[2]

    # Joypad node starten
    # rospy.set_param('joy_node/dev', '/dev/input/js' + str(js_num))
    # os.system('gnome-terminal --tab -- /bin/bash -c \
    #               "rosrun joy joy_node;\
    #               exec bash"')
    # print("JS_Mode:" +str(js_mode))
    # print("JS_Num:" +str(js_num))

    twist = Twist()
    remote_control()
