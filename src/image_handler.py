#!/usr/bin/env python
#Author: Lisandro Mayancela
#Email: lmayancela08@brandeis.edu
#Class: COSI 119a
#Assignment: Line Follower

import rospy, cv2, cv_bridge, numpy, math
from constants import *
from sensor_msgs.msg import Image
from racetrackpa.msg import img_data


# Find the center of the white spot at the specified horizontal row
def find_white_mid(image, row):
    found = False # Have we found a white spot?
    start = -1 # Index where white started
    end = -1 #Index white ended

    # image = image[0:IMG_HEIGHT, 100:IMG_WIDTH-100]

    for i in range(0, IMG_WIDTH):
        pixel = image[row, i] # Get pixel from this row at index i
        print(pixel)
        if pixel == WHITE:
            if not found:
                start = i
                found = True
            else:
                end = i 
    
    # Calculate the midpoint of the white spot
    mid = (start + end) // 2
    return mid

# Find the nearest white pixel in an image from the a specified target pixel
def locate_nearest_white(image, target):

    # We will keep track of the current closest point
    closest_point = (-1,-1)

    # In order to increase the efficiency of our algorithm, we will only search one out of every 36 pixels.
    for i in range(0,IMG_HEIGHT,6):
        for j in range(0,IMG_WIDTH,6):
            curr_pixel = (i,j) # Save the tuple value for distance calculation
            curr_pixel_color = image[i,j] # Get the color at the current pixel

            # Get distances from the current point and the last closest point to the target
            dist_curr_to_target = distance(curr_pixel, target)
            dist_closest_to_target = distance(closest_point, target)

            # If the pixel is white and the distance from it to the target is less than that from the current closest point, the current closest point is now this pixel.
            if (curr_pixel_color == WHITE) and (dist_closest_to_target > dist_curr_to_target):
                closest_point = curr_pixel

    return numpy.asarray(closest_point)


# Helper function to get the distance between two points (given as tuples)
def distance(p1, p2):
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    return (((x2-x1)**2) + ((y2-y1)**2) ** .5)

#Initialize node
rospy.init_node('image_handler')

def imgcb(msg):
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Defining range of yellow color in HSV
    upper_yellow = numpy.array([30, 255, 255])
    lower_yellow = numpy.array([20, 100, 100])

    # Delete this 
    upper_white = numpy.array([255, 10, 100])
    lower_white = numpy.array([0, 0, 25])
    # --------------------------------------


    # Threshold the HSV image to get only yellow colors
    mask = cv2.inRange(hsv, lower_white, upper_white)

    # In order to determine whether or not a line is found, I will be checking whether or not two horizontal sections (which represent 
    # the floor in front of the robot) of the image contain a section of white
    SECTION_TOP = .75 * IMG_HEIGHT
    SECTION_BOTTOM = .8 * IMG_HEIGHT
    top = find_white_mid(mask, int(SECTION_TOP))
    bottom = find_white_mid(mask, int(SECTION_BOTTOM))
    print((top,bottom))

    img_results_msg = img_data()

    if (top == -1) or (bottom == -1): # If either top or bottom are -1 then there isnt a line in front of the robot to follow so we must find the closest white to travel to
        img_results_msg.found_line = -1 #false
        img_results_msg.center = -1 
        img_results_msg.slope = 0
        img_results_msg.closest_white_pixel = locate_nearest_white(mask, (SECTION_TOP, IMG_WIDTH // 2))
        img_results_pub.publish(img_results_msg)
    else: # We have found a line to follow!
        img_results_msg.found_line = 1 #true
        img_results_msg.center = (top + bottom) // 2
        img_results_msg.slope = (bottom - top) / (SECTION_BOTTOM - SECTION_TOP)
        img_results_msg.closest_white_pixel = [-1, -1]
        img_results_pub.publish(img_results_msg)

# Subscribers and Publishers
bridge = cv_bridge.CvBridge()
img_sub = rospy.Subscriber('camera/rgb/image_raw', Image, imgcb)
img_results_pub = rospy.Publisher('img_data', img_data, queue_size = 1)

if __name__ == '__main__':
    rospy.spin()
