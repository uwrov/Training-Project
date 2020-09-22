# DO NOT RUN ME! PLEASE JUST RUN THE COMMANDS LISTED HERE ONE BY ONE
echo "Please don't run me >:("
echo "I will now enter an infinte loop! Ctrl+C to stop"
while true
do
  sleep 1
done

# Create URDF from xacro
rosrun xacro xacro wheely_boi.xacro > wb.urdf

# Start ROS and an empty Gazebo world
roslaunch gazebo_ros empty_world.launch

# Spawn our urdf model into the empty gazebo world
rosrun gazebo_ros spawn_model -file wb.urdf -urdf -model wheely_boi

# Start our keyboard controller
rosrun wb key_in.py