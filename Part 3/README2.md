# Publisher - key_input.py

We need to somehow get data to our robot. ROS uses a publisher/subscriber model, where publisher
programs will publish messages to topics, and subscriber programs will recieve data from those topics.

This is a simple publisher which will take in user input and translate that to how fast it wants a robot to go.

Defines the publisher and sets the linear and angular velocity to the default value
```
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

In our loop, we read in keyboard input from the user and update our geometry message t accordingly.
```
while not rospy.is_shutdown():
    key = curses.wrapper(getch_c)
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

    t.linear.x = close_zero(t.linear.x)
    t.angular.z = close_zero(t.angular.z)

    rospy.loginfo("Sending Command v:" + str(t.linear.x)
                    + ", y:" + str(t.angular.z))
    velocity_publisher.publish(t)
    rate.sleep()
```


The getch_c method takes input from our keyboard and returns the value
```
def getch_c(stdscr):
    # do not wait for input when calling getch
    stdscr.nodelay(1)

    # get keyboard input, returns -1 if none available
    c = stdscr.getch()
    if c != -1:
        return c
```
