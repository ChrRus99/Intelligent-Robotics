#include <ros/ros.h>
#include "nodes/NodeCActionServer.h"

int main(int argc, char** argv)
{
    // Initialize the node as an (action) server
    ros::init(argc, argv, "objects_manipulation_action_server_node");

    // Instantiate the object manipulation action server
    NodeCActionServer server;

    // Keep spinning to allow ROS to process callbacks
    ros::spin();

    return 0;
}