#include "nodes/NavigationActionServer.h"
#include "util/Point.h"
#include <assignment2/NavigationGoal.h>
#include <string>

using namespace std;
using namespace Point;

NavigationActionServer::NavigationActionServer() :
	as(nh, "navigation_topic", boost::bind(&NavigationActionServer::navigate_callback, this, _1), false),
	move_base_client("move_base", true)
{
	as.start();
}

void NavigationActionServer::navigate_callback(const assignment2::NavigationGoalConstPtr &goal)
{
    ROS_INFO("(NAVIGATION SERVER): New client connected");

	// Move the robot from its current pose to the target_pose
	bool is_navigation_succeeded = move_base_navigation(goal->target_pose);

	// If the robot successfully reached the target pose, then detect obstacles
    assignment2::NavigationResult result;

    if (is_navigation_succeeded) {
        // Send the result and finish the action
        as.setSucceeded(result);
    } else {
        // Set the action as aborted
        as.setAborted();
    }
}

bool NavigationActionServer::move_base_navigation(const geometry_msgs::Pose& target_pose) 
{
	// Populate the goal target_pose to send to the move_base_client
    move_base_msgs::MoveBaseGoal move_base_goal;
    move_base_goal.target_pose.header.frame_id = "map"; // Map frame
    move_base_goal.target_pose.header.stamp = ros::Time::now();
    move_base_goal.target_pose.pose.position = target_pose.position;
    move_base_goal.target_pose.pose.orientation = target_pose.orientation;
    
    // Calculate yaw angle (theta) using arctangent
    double theta = 2.0 * atan2(target_pose.orientation.z, target_pose.orientation.w);
    theta = theta * (180.0 / M_PI);

    ostringstream ss;
    ss << fixed << setprecision(2);
    ss << "{x=" << target_pose.position.x << ", y=" << target_pose.position.y << ", theta=" << theta << "}";

    // Wait for move_base_client to be available
    move_base_client.waitForServer();

    // Send the goal target_pose to the move_base_client and wait for the result
    ROS_INFO("(NAVIGATION SERVER): Robot has started moving towards the target pose with move_base: {x=%0.2f, y=%0.2f, theta=%0.2f}", 
        target_pose.position.x, target_pose.position.y, theta);
    feedback.status = "Robot has started moving towards the target pose with move_base: " + ss.str();
    as.publishFeedback(feedback);

    move_base_client.sendGoalAndWait(move_base_goal);

    // Chech whether the robot has successfully reached the target_pose
    if (move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    	// Robot has successfully reached the target_pose
        ROS_INFO("(NAVIGATION SERVER): Robot has successfully reached the target pose");
        feedback.status = "Robot has successfully reached the target pose";
        as.publishFeedback(feedback);

		return true;
    } else {
    	// Robot has failed to reach the target_pose
        ROS_ERROR("(NAVIGATION SERVER): Robot has failed to reach the target pose: invalid target pose (not in map)");
        feedback.status = "Robot has failed to reach the target pose: invalid target pose (not in map)";
        as.publishFeedback(feedback);

		return false;
    }
}
