#include <ros/ros.h>
#include "nodes/NavigationActionServer.h"

int main(int argc, char** argv)
{
    // Initialize the node as an (action) server
    ros::init(argc, argv, "node_navigation_action_server");

    // Instantiate the navigation action server
    NavigationActionServer server;

    // Keep spinning to allow ROS to process callbacks
    ros::spin();

    return 0;
}