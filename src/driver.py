#!/usr/bin/env python
#Author: Lisandro Mayancela
#Email: lmayancela08@brandeis.edu
#Class: COSI 119a
#Assignment: Line Follower

import rospy
from sensor_msgs.msg import LaserScan
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

def lidar_cb(msg):
    global front
    global peripheral
    ranges = msg.ranges
    minimum = msg.range_min
    inf = float('inf')
    ranges = [angle if (angle > minimum) else 0 for angle in ranges]
    ranges = [angle if (not (angle == inf)) else 0 for angle in ranges]
    front = ranges[315:]
    front.extend(ranges[:46])
    peripheral = ranges[270:]
    peripheral.extend(ranges[:90])

def obstacle():
    #find the minimum value in the lidar at the front of the robot, excluding zero, because invalid data is set to zero.
    real_min = min(i for i in front if i > 0)
    #if the shortest valid lidar reading is below 0.7, then there is an obstacle in front of the robot.
    if real_min < 0.9:
        return True
    else:
        return False

def avoid():
    twist = Twist()
    last_reading = front
    #find the nearest open space, and drive towards it.
    target = max(last_reading)
    #front includes the frontmost 90 degrees. If we're facing the max, it'll be right in the middle of these readings, so at the 45th index.
    if(last_reading.index(target) > 95):
        twist.angular.z = .50
        twist.linear.x = 0.0
    elif(last_reading.index(target) < 85):
        twist.angular.z = -0.50
        twist.linear.x = 0.0
    else:
        twist.linear.x = 0.2

    return twist

rospy.init_node('driver')
img_data_sub = rospy.Subscriber('/img_data', img_data, img_data_cb)
pid_twist_sub = rospy.Subscriber('pid_twist', Twist, pid_twist_cb)
lidar_sub = rospy.Subscriber('/scan', LaserScan, lidar_cb)
exploring_twist_sub = rospy.Subscriber('exploring_twist', Twist, exploring_twist_cb)
twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

pid_twist = Twist()
exploring_twist = Twist()

front = [0]

# Will publish either pid_twist or exploring_twist to cmd_vel
def drive(determiner):
    if obstacle():
        twist_pub.publish(avoid())
        print('avoiding')
    else:
        if determiner == 1: # We are using the PID when we are following a line
            print('line')
            twist_pub.publish(pid_twist)
        else: # If we are traversing to or looking for a line we use the explorer_twist
            twist_pub.publish(exploring_twist)

if __name__ == '__main__':
    rospy.spin()
