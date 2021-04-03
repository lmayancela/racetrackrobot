#!/usr/bin/env python
#Author: Lisandro Mayancela
#Email: lmayancela08@brandeis.edu
#Class: COSI 119a
#Assignment: Line Follower

import rospy
from geometry_msgs.msg import Twist
from racetrackpa.msg import img_data
from constants import *

def pid_twist_cb(msg):
    global pid_twist
    pid_twist = msg

def exploring_twist_cb(msg):
    global exploring_twist
    exploring_twist = msg

def img_data_cb(msg):
    line_found = msg.found_line
    drive(line_found)

rospy.init_node('driver')
img_data_sub = rospy.Subscriber('/img_data', img_data, img_data_cb)
pid_twist_sub = rospy.Subscriber('pid_twist', Twist, pid_twist_cb)
exploring_twist_sub = rospy.Subscriber('exploring_twist', Twist, exploring_twist_cb)
twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

pid_twist = Twist()
exploring_twist = Twist()

# Will publish either pid_twist or exploring_twist to cmd_vel
def drive(determiner):
    if determiner == 1: # We are using the PID when we are following a line
        twist_pub.publish(pid_twist)
    else: # If we are traversing to or looking for a line we use the explorer_twist
        twist_pub.publish(exploring_twist)

if __name__ == '__main__':
    rospy.spin()

