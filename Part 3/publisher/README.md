# Publisher - key_input.py

We need to somehow get data to our robot. ROS uses a publisher/subscriber model, where publisher
programs will publish messages to topics, and subscriber programs will receive data from those topics.

This is a simple publisher which translates user input into force applied onto the robot.

First thing we'll want to do when writing Python scripts is to specify what Python interpreter we want the program to use. We'll do this by just adding a comment at the top of the file:
```
#!/usr/bin/env python3
```
This comment isn't necessary on every computer, but you should add it in if there are multiple versions of Python installed and you want to make your Python script executable.


Defines the publisher and sets the linear and angular velocity to the default value
```Python
def move():
    # Create a new node
    velocity_publisher = rospy.Publisher('/wheely_boi/wheely_boi/cmd', Twist, queue_size=10)
    rospy.init_node('wheely_boi', anonymous=True)
    t = Twist()
    rate = rospy.Rate(10)

    t.linear.x = 0
    t.angular.z = 0
```

Our publisher constantly advertises the state of the robot, so we use a while loop.

In our loop, we read in keyboard input from the user (using `getch_c`) and update our geometry message `t` accordingly.
```Python
while not rospy.is_shutdown():

    # gather user input
    key = curses.wrapper(getch_c)

    # Handle user input
    if (key == ord('w')):
        t.linear.x = min(t.linear.x + 0.1, 1.0)
    elif (key == ord('a')):
        t.angular.z = min(t.angular.z + 0.1, 1.0)
    elif (key == ord('s')):
        t.linear.x = max(t.linear.x - 0.1, -1.0)
    elif (key == ord('d')):
        t.angular.z = max(t.angular.z - 0.1, -1.0)
    elif (key == ord('q')):
        t.linear.x = 0
        t.angular.z = 0

    # Package input into message
    t.linear.x = close_zero(t.linear.x)
    t.angular.z = close_zero(t.angular.z)

    # Publish the message
    rospy.loginfo("Sending Command v:" + str(t.linear.x)
                    + ", y:" + str(t.angular.z))
    velocity_publisher.publish(t)
    rate.sleep()
```