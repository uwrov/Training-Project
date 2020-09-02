# Create URDF from xacro
rosrun xacro xacro wheely_boi.xacro > wb.urdf

# Start ROS and an empty Gazebo world
roslaunch gazebo_ros empty_world.launch

# Spawn our urdf model into the empty gazebo world
rosrun gazebo_ros spawn_model -file wb.urdf -urdf -model wheely_boi

# Start our keyboard controller
rosrun wb key_in.py