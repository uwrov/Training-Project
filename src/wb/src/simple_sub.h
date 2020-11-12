#ifndef SIMPLE_SUB_H_
#define SIMPLE_SUB_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Target function our subscriber will use to
// process information gathered from topic
void chatterCallback(const geometry_msgs::Twist&);

#endif  //SIMPLE_SUB_H_