Robot Plugin

Load Method
```
virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    ...
}
```
- Here, we are overriding the ModelPlugin's Load function
- The method is called every time the plugin is loaded
- Parameter 1: The pointer to our robot's physics model
- Parameter 2: The pointer to our robot's SDF (this is like an interpreted URDF)

Components of the Load Method
```
virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
     {
       _m = _model;

       // Checks if ROS node master has been loaded
       if(!ros::isInitialized)
       {
         ROS_FATAL_STREAM("ROS node for Gazebo not established. Plugin failed");
         return;
       }
       ROS_INFO("Wheely Boi Plugin Loaded");

       // Setting friction, since it is not loaded from the URDF
       _m->GetJoint("jointR")->SetParam("friction", 0, 0.0);
       _m->GetJoint("jointL")->SetParam("friction", 0, 0.0);

       std::string name = _m->GetName();

       // Spawning the model
       ROS_INFO("Spawning node: %s", name.c_str());
       node.reset(new ros::NodeHandle(name.c_str()));
       ROS_INFO("Node spawned.");

       // Initializing the ROS subscriber
       sub = new ros::Subscriber();
       std::string subName = name + "/cmd";  // /wheely_boi/wheely_boi/cmd
       *sub = node->subscribe(subName, 1, &WBPlugin::onCmd, this);  // takes care of the actual subscribing

       _w = _m->GetWorld();
       lastSimTime = _w->GetSimTime();

       lc_V = lc_Y = 0;
       lc_T = lastSimTime;

       // Initializing prior positions and rotations
       pRot = _m->GetWorldPose().rot;
       pPos = _m->GetWorldPose().pos;

       // Binding the onUpdate function to a callback every nanosecond
       this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&WBPlugin::onUpdate, this, _1));
      }
}

```
- The if statement ensures that the ROS master node has been launched. It will terminate the program otherwise.
- Some initial states that are not loaded from the URDF can be set in the Load method (ex: friction).
- The Load method is also where we spawn in the model, initialize the ROS subscriber and initialize initial positions and rotations.
- In the Load method, we have to bind the onUpdate function to be called every nanosecond.


*onUpdate Method*
Updates the movement every nanosecond
- create a timeout for every nanosecond, since a real microcontroller does not process at that rate
  - get current time, define length of wait period and timeout
  - if currentTime - lastCheckedTime < waitTime, return (exit method)
  - if currentTime - timeOfLastCommand > timeout, set timeOfLastCommand = currentTime and set veloccity and yaw values to 0

- Gets the current rotation and position information
```
math::Quaternion rot = _m->GetWorldPose().rot;
math::Vector3 pos = _m->GetWorldPose().pos;
```
and the distance travelled in the last timestamp
```
math::Vector3 dist = pos - pPos;
```

- Determine the desired speed by multiplying the maximum linear velocity and the last given velocity command
```
double desVel = lc_V * MAX_LIN_V;
```

- Get the current rotation values
```
double roll = rot.GetRoll();
double pitch = rot.GetPitch();
double yaw = rot.GetYaw();
```
and the change in angular velocities by subtracting the previous rotation values
```
double yawV = yaw - pRot.GetYaw();
double pitchV = pitch - pRot.GetPitch();
```

- Determine if the velocity is positive or negative (using math I don't understand), indicating whether the robot travels forward or backward compared to its "front"
```
double velSign = SIGN(dist.x * cos(yaw) + dist.y * sin(yaw));
double vel = velSign * sqrt(dist.x * dist.x + dist.y * dist.y);
```

- Determine how much we can yaw without the robot falling over
```
double maxYV = NZ(((MAX_LIN_V-MAG(vel))/(MAX_LIN_V))*MAX_YAW_V, 0.00001);
```