#ifndef WB_PLUGIN_H
#define WB_PLUGIN_H

// Include relevant libraries
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>

#include <string>

// Used messages
#include "geometry_msgs/Twist.h"

// Figure out if a is positive or negative
#define SIGN(a) (a >= 0 ? 1.0 : -1.0)

// Find the absolute value of a
#define MAG(a) (a * SIGN(a))

// If a is 0, use value b instead
#define NZ(a, b) (a == 0 ? b : a)

// If the magnitude is above b, use b with the sign of a
#define MAX_MAG(a, b) (MAG(a) > b ? b * SIGN(a) : a)

// If the magnitude is below b, use b with the sign of a
#define LES_MAG(a, b) (MAG(a) <= MAG(b) ? a : b)

// Converts from -pi to pi to 0 to 2pi
#define ZTP(a) (a < 0 ? a + 2.0 * PI : a)
// Converts form 0 to 2pi to -pi to pi
#define NPP(a) (a > PI ? 2.0 * PI - a : a)

// This is the maximum effort our motors can put out
// And the max the simulation will use
#define MAX_MOTOR_TORQUE 0.30

// This is the maximum value we will scale our desired effort to
#define EFFORT_MAX_MAG 0.49

// This is the maximum effort we will scale our turning effort to
#define YAW_MAX_EFFORT 0.05

// This is the highest magnitude pitch from standing that we want to
// ever try to obtain (in radians)
#define UPPER_PITCH_BOUND 0.050

// Maximum velocities we will aim for (in units/(LOOPTIME ms))
#define MAX_LIN_V 0.05
#define MAX_YAW_V 0.03
#define MAX_PITCH_V 0.01

// Pi is a useful number to have
#define PI 3.14159265359

// Realtime controllers only update on a set loop, so we don't want
// to pretend that our edumip will be able to update faster than it can
// so we have some logic to restrict the processing speed of our edumip
// simulation.
#define LOOPTIME_MS 20.0

// A lot of the time required is in nanoseconds, but our controller works
// on milisecond times not nanosecond.
#define MS_TO_NS(a) a * 1000000.0

// Gazebo Plugin which will read messages from /model_name/model_name/cmd
// and make a robot simulated in Gazebo move accordingly

namespace gazebo {
// Create class WBPlugin which extends ModelPlugin
class WBPlugin : public ModelPlugin {
 public:
  WBPlugin() {}   // Empty constructor

  // Called when we spin() to deal with
  // given geometry messages on stack
  void onCmd(geometry_msgs::Twist t);

  // Called every nanosecond of sim-time,
  // contains all the calculations which determine how
  // wheely_boi moves around.
  void onUpdate(const common::UpdateInfo &inf);

  // Runs once when the model is first loaded in
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

 private:
  // This pointer gives us access to all of the state information
  // of our edumip model
  physics::ModelPtr _m;

  // This pointer gives us access to all of the state information
  // of the simulated world.
  physics::WorldPtr _w;

  // Timestamp of the last processing cycle of our edumip.
  // Used to help us simulate the limited processing on a realtime
  // controller.
  common::Time lastSimTime;

  // Pointer to event callback for our onUpdate function
  // Called every nanosecond of the simulation.
  event::ConnectionPtr updateConnection;

  // Our ROS pointers.
  std::unique_ptr<ros::NodeHandle> node;
  ros::Subscriber *sub;

  // Prior positions and rotations to calculate velocities.
  ignition::math::Quaternion<double> pRot;
  ignition::math::Vector3<double> pPos; 

  // Last given velocity and yaw commands.
  double lc_V;
  double lc_Y;
  // Timestamp of our last recieved command.
  common::Time lc_T;
};
}

#endif  // WB_PLUGIN_H