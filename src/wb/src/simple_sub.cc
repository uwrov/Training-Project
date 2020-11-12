#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include "./simple_sub.h"

int main(int argc, char **argv) {
    // Step 1: Set up the node

    // Set up subscriber
    std::string subName = "/wheely_boi/wheely_boi/cmd";
    // Step 2: Attach a subscriber to this node and point to chatterCallback

    // Don't go past this point so long as ros node is alive
    // Uncommnet this line when you're done
    // ros::spin();


    // Return success
    return 0;
}

void chatterCallback(const geometry_msgs::Twist& t) {
    ROS_INFO("Linear: (%f, %f)", t.linear.x, t.linear.z);
    ROS_INFO("Angular: (%f, %f)\n", t.angular.x, t.angular.z);
}
