#!/usr/bin/env python3
import rospy
import curses
from geometry_msgs.msg import Twist

# Send messages to /wheely_boi/wheely_boi/cmd based on
# keyboard input from user
def move():
    # Create a new node
    rospy.init_node('wheely_boi', anonymous=False)
    
    # Create a publisher and point it to a topic
    velocity_publisher = rospy.Publisher('/wheely_boi/wheely_boi/cmd', Twist, queue_size=10)
    rate = rospy.Rate(10)
    
    # Initailize object we'll be publishing
    t = Twist()
    t.linear.x = 0
    t.angular.z = 0
    # Collecting User Input

    # STEP 1: Create an infinite loop that will take in keyboard input using getch_c,
    #         convert user input into an appropriate geometry message,
    #         publish the message, then sleep for a short time.
    #
    #         Here are the user inputs:
    #         w - increment linear x direction force by +0.1
    #         s - decrement linear x direction force by -0.1
    #         a - decrement angular z direction force by -0.1
    #         d - increment angular z direction force by +0.1
    #         q - set all force to 0
   

# Grab input from user (nonblocking)
# Returns the key the user pressed
def getch_c(stdscr):
    # do not wait for input when calling getch
    stdscr.nodelay(1)

    # get keyboard input, returns -1 if none available
    c = stdscr.getch()
    if c != -1:
        return c


if __name__ == '__main__':
    move()

