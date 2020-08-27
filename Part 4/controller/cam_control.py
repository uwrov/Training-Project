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


# Do the main loop
def main():
    # Setup
    src = 'http://192.168.0.11:8080/video/mjpeg'
    wait_time = 1

    network = neural.Net()
    network.load_state_dict(torch.load('../src/tools/results/model.pth'))

    velocity_publisher = rospy.Publisher('/wheely_boi/wheely_boi/cmd',
                                         Twist, queue_size=10)
    rospy.init_node('wheely_boi', anonymous=True)
    t = Twist()
    rate = rospy.Rate(1)

    stream = VideoStream(src)
    stream.start()

    while not rospy.is_shutdown():
        frame = stream.frame
        cv2.imshow('frame', frame)
        
        publish(find_label(frame, network), t, velocity_publisher)
        
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