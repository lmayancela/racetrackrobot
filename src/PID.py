#!/usr/bin/env python
#Author: Lisandro Mayancela
#Email: lmayancela08@brandeis.edu
#Class: COSI 119a
#Assignment: Line Follower

import rospy
from geometry_msgs.msg import Twist
from line_follower_pa_lisandro.msg import img_data
from constants import *

# Components for our PID controller
p_component = 0
i_component = 0
d_component = 0

# Array of the past 10 errors
error_log = [0.0] * 10

def img_data_cb(msg):
    global p_component, d_component, i_component
    found_line = msg.found_line
    # This node will only do something if a line is found. When the line hasn't been found yet the explorer will take over
    if found_line == 1:
        # Get the values for center and slope from the message
        center = msg.center
        error = (IMG_WIDTH // 2) - center
        update_error_log(error)

        p_component = error # Our P component will be the error between the center of the line that was given and the center of the image
        i_component = sum(error_log) / 10 # Our integral component will be the average error taken from the past 10 errors
        d_component = msg.slope # Our D component will be the slope

# Save the first 9 elements of the error log and add the new error to the front
def update_error_log(error):
    global error_log
    # Shift all elements to the right
    for i in range(0, 9):
        error_log[i+1] = error_log[i]
    error_log[0] = error

# Initialize node
rospy.init_node('PID')
twist_pub = rospy.Publisher('pid_twist', Twist, queue_size=1)
img_data_sub = rospy.Subscriber('/img_data', img_data, img_data_cb)
t = Twist()
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    t.linear.x = LINEAR_VEL
    pid_factor = (P_CONSTANT * p_component) + (I_CONSTANT * i_component) + (D_CONSTANT * d_component)
    t.angular.z = ANGULAR_VEL * pid_factor
    twist_pub.publish(t)
    rate.sleep()

