#!/usr/bin/env python
#Author: Lisandro Mayancela
#Email: lmayancela08@brandeis.edu
#Class: COSI 119a
#Assignment: Line Follower

import rospy
from geometry_msgs.msg import Twist
from racetrackpa.msg import img_data
from constants import *

def img_data_cb(msg):
    found_line = msg.found_line
    # This node will only do something if the robot hasn't found a line to follow yet. The PID controller will handle when a line is found
    if found_line == -1:
        closest_white_point = msg.closest_white_pixel
        explore_to_point(closest_white_point)

# Initialize node and subs/pubs
rospy.init_node('explorer')
img_data_sub = rospy.Subscriber('/img_data', img_data, img_data_cb)
twist_pub = rospy.Publisher('exploring_twist', Twist, queue_size=1)

# Determine linear and angular velocity values based on where the point is
def explore_to_point(p):
    t = Twist()
    px = p[0]
    py = p[1]

    # These values are used as limits for how far off center a section of white can be before needing to rotate towards it 
    LEFT_LIMIT = IMG_WIDTH // 2 - 160
    RIGHT_LIMIT = IMG_WIDTH // 2 + 160
    HEIGHT_LIMIT = IMG_HEIGHT * 70

    # If the x or y coordinates are negative then there are no yellow lines in view so we must traverse until something is found
    if px < 0 or py < 0:
        t.linear.x = LINEAR_VEL * .2
        t.angular.z = ANGULAR_VEL * .8
    elif px < LEFT_LIMIT:
        # If the point is too far left, rotate to the left.
        t.linear.x = 0
        t.angular.z = ANGULAR_VEL
    elif px > RIGHT_LIMIT:
        # If the point is too far right, rotate to the right.
        t.linear.x = 0
        t.angular.z = ANGULAR_VEL * -1
    elif py < HEIGHT_LIMIT:
        # If the point is too far away then move towards it
        t.linear.x = LINEAR_VEL
        t.angular.z = 0
    else:
        # If this statement is reached then we are looking at a line that is in front of us and in view
        # The job of this node is done and we wait until PID takes over.
        t.linear.x = 0
        t.angular.z = 0

    # Publish this twist
    twist_pub.publish(t)

if __name__ == '__main__':
    rospy.spin()


    
