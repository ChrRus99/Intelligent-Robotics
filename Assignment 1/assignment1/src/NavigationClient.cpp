#include "NavigationClient.h"

using namespace std;

NavigationClient::NavigationClient() : ac("navigation", true)
{
	// Wait for the action server to start
    ac.waitForServer();
}

void NavigationClient::sendGoal(const geometry_msgs::Pose& target_pose) {
    assignment1::NavigationGoal goal;
    goal.target_pose = target_pose;
    
    ac.sendGoal(goal, 
        boost::bind(&NavigationClient::doneCallback, this, _1, _2),
    	boost::bind(&NavigationClient::activeCallback, this),
        boost::bind(&NavigationClient::feedbackCallback, this, _1)
    );
}

void NavigationClient::doneCallback(const actionlib::SimpleClientGoalState& state, const assignment1::NavigationResultConstPtr& result) {
	// Process the result received from the server
    ROS_INFO("PROGRAM STATUS (CLIENT): Navigation completed");

    // Print result of obstacle positions
    if(result->obstacle_positions.empty()) {
        ROS_INFO("PROGRAM STATUS (CLIENT): No obstacle detected");
    }
    
    for (const auto& obstacle : result->obstacle_positions) {
        ROS_INFO("PROGRAM STATUS (CLIENT): Obstacle detected position: [x=%f, y=%f]", obstacle.position.x, obstacle.position.y);
    }

    // Terminate the ROS node and exit the program
    ROS_INFO("PROGRAM STATUS (CLIENT): Client process is terminated");
    ros::shutdown();
    exit(0);
}

void NavigationClient::activeCallback() {
	ROS_INFO("PROGRAM STATUS (CLIENT): Activated navigation to target_pose");
}

void NavigationClient::feedbackCallback(const assignment1::NavigationFeedbackConstPtr& feedback) {
	// Process feedback received from the server
    ROS_INFO("PROGRAM STATUS (CLIENT): Feedback received: %s", feedback->status.c_str());
}
