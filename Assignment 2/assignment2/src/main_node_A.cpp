#include "nodes/NodeA.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    // Initialize node_A
    ros::init(argc, argv, "node_A");

    // Get extra points flag from the command line
    bool extra_points_flag = false;
    if(!ros::param::get("~extra_points_flag", extra_points_flag)) {
        ROS_ERROR("PROGRAM STATUS (SERVER): Missed or invalid extra_points_flag: set extra_points_flag=false by default");
        // program continues...
    }

    ROS_INFO("PROGRAM STATUS (SERVER): Flag extra points set to: %d", extra_points_flag);

    // Instantiate NodeA
    NodeA nodeA(extra_points_flag);
    nodeA.execute_pick_up_delivery_pipeline();

    // Keep spinning to allow ROS to process callbacks
    ros::spin();

    return 0;
}