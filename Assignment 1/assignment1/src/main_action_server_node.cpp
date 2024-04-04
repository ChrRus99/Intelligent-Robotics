#include <ros/ros.h>
#include "NavigationServer.h"

int main(int argc, char** argv)
{
    // Initialize the node as an (action) server
    ros::init(argc, argv, "navigation_server_node");

    // Get control law navigation flag from the command line
    bool CL_flag = false;
    if(!ros::param::get("~CL_flag", CL_flag)) {
        ROS_ERROR("PROGRAM STATUS (SERVER): Missed or invalid CL_flag: setted CL_flag=false by default");
        // program continues...
    }

    ROS_INFO("PROGRAM STATUS (SERVER): Flag narrow corridor control law set to: [CL_flag=%d]", CL_flag);

    // Send the CL_flag to the server
    NavigationServer server(CL_flag);

    ros::spin();

    return 0;
}