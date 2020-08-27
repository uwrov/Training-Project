# Part 1 - The Big Picture

## Robot Model

Our ROV consists of 3 different layers. Lowest to highest level:
1. Motor Code which tells the motors to move with certain voltages (speed).
2. Onboard Raspberry Pi which tells the Arduino what to do and gives a live video feed to the surface.
3. Surface Computer which we use to give commands to ROV.

Mostly we'll be working between layers 2 and 3: creating computer vision algorithms to complete tasks.

To make our lives a little simpler, we'll be using ROS to handle all the communication between layers.

## ROS
[ROS](http://wiki.ros.org/) is the Robot Operating System. Like any other operating system, it helps abstract a lot of the nitty gritty logistic details from you - the programmer.

ROS helps different programs communicate with each other by providing nodes. Publishers can put out important data to nodes, and subscribers can read in that data. There's a lot of networking invovled in getting data from one place to another, and ROS provides an abstraction for that.

ROS also moonlights as a package manager. It supports both Python and C++, so it has a lot of versatilty. A big bonus of using ROS that the nodes are totally independent of programming languages - so we can have one person write a publisher in Python and another write a subscriber in C++.

We'll be writing most of the code in this tutorial in Python, but we could have written it in C++.

Underwater simulator
https://github.com/clydemcqueen/orca2