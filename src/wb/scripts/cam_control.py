import cv2
import numpy as np
import torch
import torchvision
import time

import tools.neural as neural
from tools.cam_util import find_label

import rospy
from geometry_msgs.msg import Twist

from threading import Thread
from tools.VideoStream import VideoStream


# Publishes messages to /wheely_boi/wheely_boi/cmd
# Based on an incoming camera feed
def main():
    # Setup
    # STEP 1: Assign your video stream source
    # (this will actually be different for everyone)
    src = ''
    wait_time = 1

    network = neural.Net()
    network.load_state_dict(torch.load('../src/tools/results/model.pth'))

    # STEP 2: Define and initialize a Publisher node which
    # will publish to /wheely_boi/wheely_boi/cmd
    velocity_publisher = None


    t = Twist()
    rate = rospy.Rate(1)

    stream = VideoStream(src)
    stream.start()

    while not rospy.is_shutdown():
        frame = stream.frame
        cv2.imshow('frame', frame)

        # STEP 3: use find_label to read the number written in the camera feed,
        # then invoke publish() to format and publish the message 
        
        
        rate.sleep()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stream.join()
            break

    cv2.destroyAllWindows()


# Publish signals to our created ROS topic
def publish(signal, t, velocity_publisher):
    if (signal == 8):
        t.linear.x = min(t.linear.x + 0.1, 1.0)
    elif (signal == 4):
        t.angular.z = min(t.angular.z + 0.1, 1.0)
    elif (signal == 6):
        t.linear.x = max(t.linear.x - 0.1, -1.0)
    elif (signal == 2):
        t.angular.z = max(t.angular.z - 0.1, -1.0)
    else:
        t.linear.x = 0
        t.angular.z = 0
    
    t.linear.x = round(t.linear.x, 1)
    t.angular.z = round(t.angular.z, 1)

    rospy.loginfo("Sending Command v:" + str(t.linear.x) 
                    + ", y:" + str(t.angular.z))
    velocity_publisher.publish(t)


if __name__ == '__main__':
    main()