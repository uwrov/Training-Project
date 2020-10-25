# Part 1 - The Big Picture

## Robot Model and Intro

Our ROV consists of 3 different layers.
1. Internal - Code which deals with motors and sensors on ROV directly. 
2. Onboard - Interprets commands from Surface, relays them to Internal. Also gives sensor data to Surface.
3. Surface - Interface we use to give the robot commands. 

If it helps, think about our robot like a company:
1. Workers who carry tasks out.
2. Bosses who tell the workers what to do, recieve orders from shareholders. Also need to report progress back go shareholders.
3. Shareholders who tell the bosses what they want the company to achieve.

We'll be developing each of these layers throughout the next two quarters, but this tutorial will focus mainly on layers 2 and 3.

By the end of this project you should have a basic understanding of ROS and robot simulations.

## ROS
[ROS](http://wiki.ros.org/) is the Robot Operating System. Like any other operating system, it helps abstract a lot of the nitty gritty logistic details from you - the programmer.

ROS does this by providing two things: a package manager which organizes our code and a simple-to-understand model for program-program communication.

### Packages
ROS allows us to create packages which hold related code together. There are two reasons why ROS packages are useful:

1. We can start a program from any package from a terminal at any location with `rosrun`.
2. We should be able to transfer packages between ROS environments and have the same functionality.

### Communication
ROS abstracts networking away from the programmer using nodes and topics.

- We launch a ROS __node__ in any program that communicates. In this particular model, each node is designated as either a __publisher__ or a __subscriber__
- __Publisher__ nodes will push out messages to __topics__, which one or more __subscribers__ can pull data from.

This kind of model is called a publish/subscribe model. ROS also provides a way to use a service/client model, which resembles a server-client connection. We'll be using the publish/subscribe model in this tutorial project.

A nice benefit of handling program-program communication using a publish/subscribe model is that our communicating programs don't even have to be in the same language!
ROS mainly supports C++ and Python, but there are libraries which add support for other languages like Java and Ruby.

We'll be focusing on C++ and Python in this project (mainly Python). You're welcome to approach tasks in the future in whatever language you want, but our club will only be able to give useful feedback if you're using C++ or Python.
