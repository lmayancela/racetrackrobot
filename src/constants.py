#Author: Lisandro Mayancela
#Email: lmayancela08@brandeis.edu
#Class: COSI 119a
#Assignment: Line Follower

# We are defining some constants that may be useful in our other ros nodes

# Dimensions of the image
IMG_HEIGHT = 1080
IMG_WIDTH = 1920

# Max angular and linear velocities
# ANGULAR_VEL = 0.4
# LINEAR_VEL = 0.35

ANGULAR_VEL = 0.75
LINEAR_VEL = 1.0

# Constants for PID controller
P_CONSTANT = 0.001
I_CONSTANT = .0003
D_CONSTANT = 0.2

# Values for black and white in the masked image
WHITE = 255
BLACK = 0
