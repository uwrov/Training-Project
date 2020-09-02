# Build a Robot with ROS
Welcome to UWROV! Over the course of this project we'll do our best to teach you what you need to know to develop for the ROV.

[Solution Repo](https://github.com/uwrov/ros_solution)

In this project, we'll have you complete the files:
- `src/wb/urdf/wheely_boi.xacro`
- `src/wb/plugins/wb_plugin.cc`
- `src/wb/src/tools/cam_util.py`
- `src/wb/scripts/cam_control.py`

To complete the files, you'll want to write code in areas denoted with `STEP N:`. You're encouraged to read through our annotated copies of the code files in the Parts 1-4 folders.

Most steps only ask for one or two lines of code, but try to put in extra effort to understand what those lines of code do!

When you're done, put the `src` directory into a fresh ROS workspace in your UWROV VM installation, then make it using `catkin_make` at the workspace level.

Next, make sure to `source devel/setup.sh`, then cd into `src/wb/urdf`.
Then run the commands from `run.sh` to get wheely_boi working with keyboard controls!

## Part 1 - The Big Picture
In this part you will learn about:
  - A big picture model of how our ROV is controlled
  - What ROS is, and why we want to learn about it
  - What this project will be

## Part 2 - Building a Basic Robot
In this part you will learn about:
  - Unified Robot Description Format (URDF)
  - Xacro
  - Gazebo

## Part 3 - Keyboard Control
In this part you will learn about:
  - ROS Subscribers/Listeners
  - Gazebo Plugins

## Part 4 - Camera Control
In this part you will learn about:
  - Neural Networks
  - OpenCV

## Part 5 - What's Next?
You've finished the robot! Reflect on what you've learned and how cool you are.
There's still a whole lot to learn about though! Here are some areas we'll need to continue improving on:
  - Computer vision for the autonomy tasks
  - Creating an interface to control our actual ROV
