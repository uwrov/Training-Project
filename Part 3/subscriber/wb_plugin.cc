#ifndef EDUMIP_CODY_PLUGIN
#define EDUMIP_CODY_PLUGIN

// Include our header
#include "./wb_plugin.h"

// Relevant libraries
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// ROS included so we can communicate back and forth
#include <ros/ros.h>

#include <string>

// Used messages
#include "geometry_msgs/Twist.h"

namespace gazebo {

void WBPlugin::onCmd(geometry_msgs::Twist t) {
  // Update our last given commands.
  lc_T = _w->SimTime();
  lc_V = t.linear.x;
  lc_Y = t.angular.z;
}

void WBPlugin::onUpdate(const common::UpdateInfo &inf) {
  // A real microcontroller doesn't process every nanosecond, 
  // so we're only gonna process after X milliseconds to simulate 
  // the firmware we are gonna work with
  common::Time cSimTime = _w->SimTime();
  common::Time wait(0, MS_TO_NS(LOOPTIME_MS));
  common::Time commandTimeout(5, MS_TO_NS(0));
  if (cSimTime - lastSimTime < wait)
  {
    return;
  }
  lastSimTime = cSimTime;

  if (cSimTime - lc_T > commandTimeout)
  {
    lc_T = cSimTime;
    lc_Y = lc_V = 0;
  }

  // Get current rotation and position information of our Edumip
  ignition::math::Quaternion<double> rot = _m->WorldPose().Rot();
  ignition::math::Vector3<double> pos = _m->WorldPose().Pos();

  // Distance travelled in last timestep
  ignition::math::Vector3<double> dist = pos - pPos;

  // Translate our inputted control into the desired speed.
  double desVel = lc_V * MAX_LIN_V;

  // Current rotations
  double roll = rot.Roll();
  double pitch = rot.Pitch();
  double yaw = rot.Yaw();

  // Angular velocities
  double yawV = yaw - pRot.Yaw();
  double pitchV = pitch - pRot.Pitch();

  // Is our velocity forward or backwards compared to the "front"
  // of the robot (rotate cartesian coordinates by yaw and then
  // see if X is positive)
  double velSign = SIGN(dist.X() * cos(yaw) + dist.Y() * sin(yaw));
  double vel = velSign * sqrt(dist.X() * dist.X() + dist.Y() * dist.Y());

  // How much we can yaw without falling depends on our velocity.
  double maxYV = NZ(((MAX_LIN_V-MAG(vel))/(MAX_LIN_V))*MAX_YAW_V, 0.00001);

  // The more we pitch the faster we accellerate. So we control velocity
  // with pitch
  double desPitch = ((desVel - vel) / (MAX_LIN_V)) * UPPER_PITCH_BOUND; 

  // Normalize pitch, and then figure out how much correction we need on
  // the wheels in order to get the right pitch
  double normPitchDiff = (desPitch - pitch) / (PI / 3.25);
  double correctionEffort = normPitchDiff * 
    ((SIGN(normPitchDiff) * MAX_PITCH_V - pitchV) / 
        (SIGN(normPitchDiff) * MAX_PITCH_V)) * EFFORT_MAX_MAG;

  // Figure out our yaw control
  double yawDif = lc_Y;

  // Effort on wheels affects yaw accelleration, 
  // so we control that to control yaw velocity.
  double yawCorrection = ((yawDif*maxYV-yawV)/(maxYV))*(YAW_MAX_EFFORT); 

  // Send motor commands
  double rEff = MAX_MAG(correctionEffort-yawCorrection, MAX_MOTOR_TORQUE);
  double lEff = MAX_MAG(correctionEffort+yawCorrection, MAX_MOTOR_TORQUE);
  _m->GetJoint("jointR")->SetForce(0,rEff);
  _m->GetJoint("jointL")->SetForce(0,lEff); 

  // Update prior position and rotation info.
  pRot = rot;
  pPos = pos;

  // Take the commands off of the stack.
  ros::spinOnce();
}

void WBPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  _m = _model;

  if(!ros::isInitialized)
  {
    ROS_FATAL_STREAM("ROS node for Gazebo not established. Plugin failed");
    return;
  }
  ROS_INFO("Wheely Boi Plugin Loaded");

  // For some reason the joint friction isn't loaded from the URDF
  // so we have to manually update them here
  _m->GetJoint("jointR")->SetParam("friction", 0, 0.0);
  _m->GetJoint("jointL")->SetParam("friction", 0, 0.0);

  std::string name = _m->GetName();

  // Spawn our node
  ROS_INFO("Spawning node: %s", name.c_str());
  node.reset(new ros::NodeHandle(name.c_str())); 
  ROS_INFO("Node spawned.");

  // Initialize our ROS subscriber
  sub = new ros::Subscriber();
  std::string subName = name + "/cmd";
  *sub = node->subscribe(subName, 1, &WBPlugin::onCmd, this);

  _w = _m->GetWorld();
  lastSimTime = _w->SimTime();

  lc_V = lc_Y = 0;
  lc_T = lastSimTime;

  // Initialize our prior positions and rotations.
  pRot = _m->WorldPose().Rot();
  pPos = _m->WorldPose().Pos();

  // Bind our onUpdate function to a callback that 
  // happens every nanosecond
  this->updateConnection = 
      event::Events::ConnectWorldUpdateBegin(boost::bind(&WBPlugin::onUpdate,
                                                         this, _1));
}

// Does all of the work to set up the rest of the plugin functionality.
GZ_REGISTER_MODEL_PLUGIN(WBPlugin)

}


#endif  // EDUMIP_CODY_PLUGIN