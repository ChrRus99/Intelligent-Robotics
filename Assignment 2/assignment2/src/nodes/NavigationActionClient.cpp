#include "nodes/NavigationActionClient.h"

using namespace std;
using namespace Point;
using namespace PointParser;

NavigationActionClient::NavigationActionClient() : ac("navigation_topic", true)
{
	// Wait for the action server to start
    ac.waitForServer();
}

void NavigationActionClient::send_goal(const Pose& target_pose) 
{
    // Build the goal
    assignment2::NavigationGoal goal;
    goal.target_pose = build_geometry_msgs_pose(target_pose);
    
    // Send the goal
    ac.sendGoal(goal, 
        boost::bind(&NavigationActionClient::done_callback, this, _1, _2),
    	boost::bind(&NavigationActionClient::active_callback, this),
        boost::bind(&NavigationActionClient::feedback_callback, this, _1)
    );

    // Initialize the navigation flag as in progress
    is_navigation_finished = false;
}

void NavigationActionClient::done_callback(const actionlib::SimpleClientGoalState& state, const assignment2::NavigationResultConstPtr& result) 
{
	// Process the result received from the server
    ROS_INFO("(NAVIGATION CLIENT): Navigation completed");

    // Print the result of obstacle positions
    /*
    if(result->obstacle_positions.empty()) {
        ROS_INFO("(NAVIGATION CLIENT): No obstacle detected");
    }
    
    for (const auto& obstacle : result->obstacle_positions) {
        ROS_INFO("(NAVIGATION CLIENT): Obstacle detected position: [x=%f, y=%f]", obstacle.position.x, obstacle.position.y);
    }
    */

    // Set the navigation flag as terminated
    is_navigation_finished = true;

    // Terminate the ROS node and exit the program
    ROS_INFO("(NAVIGATION CLIENT): Client process is terminated");
    //ros::shutdown();
    //exit(0);
}

void NavigationActionClient::active_callback() 
{
	ROS_INFO("(NAVIGATION CLIENT): Activated navigation to target_pose");
}

void NavigationActionClient::feedback_callback(const assignment2::NavigationFeedbackConstPtr& feedback)
{
	// Process feedback received from the server
    ROS_INFO("(NAVIGATION CLIENT): Feedback received: %s", feedback->status.c_str());
}