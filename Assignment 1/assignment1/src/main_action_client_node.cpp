#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <cmath>
#include "NavigationClient.h"

int main(int argc, char** argv)
{
    // Initialize the node as an (action) client and 
    ros::init(argc, argv, "navigation_client_node");
    
    // Get the target pose from the command line
    geometry_msgs::Pose target_pose;
    double x, y, theta;

    if(!ros::param::get("~x", x)) {
        ROS_ERROR("PROGRAM STATUS (CLIENT): Failed to get coordinate x of target robot pose");
        return 1;
    }
    
    if(!ros::param::get("~y", y)) {
        ROS_ERROR("PROGRAM STATUS (CLIENT): Failed to get coordinate y of target robot pose");
        return 1;
    }

    if(!ros::param::get("~theta", theta)) {
        ROS_ERROR("PROGRAM STATUS (CLIENT): Failed to get coordinate theta of target robot pose");
        return 1;
    }

    ROS_INFO("PROGRAM STATUS (CLIENT): Received target pose parameters: [x=%f, y=%f, theta=%f]", x, y, theta);

    // Calculate the z component of the orientation quaternion
    theta = theta * (M_PI / 180.0); // degree to rad
    double z = sin(theta / 2.0);
    double w = cos(theta / 2.0);

    // Populate target_pose with the user input
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = z;
    target_pose.orientation.w = w;

    // Send the target_pose goal to the server
    NavigationClient client;
    client.sendGoal(target_pose);

    ros::spin();

    return 0;
}