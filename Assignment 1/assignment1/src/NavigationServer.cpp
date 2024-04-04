#include "NavigationServer.h"
#include "util/Point.h"
#include <assignment1/NavigationGoal.h>
#include <string>

using namespace std;
using namespace Point;

NavigationServer::NavigationServer(bool CL_flag) :
	as(nh, "navigation", boost::bind(&NavigationServer::navigate_callback, this, _1), false),
	move_base_client("move_base", true)
{
	as.start();

    // Set the control law flag
    this->CL_flag = CL_flag;
}

void NavigationServer::navigate_callback(const assignment1::NavigationGoalConstPtr &goal)
{
    ROS_INFO("PROGRAM STATUS (SERVER): New client connected");

    // Move the robot through the narrow corridor (if CL_flag=true)
    control_law_navigation(CL_flag);

	// Move the robot from its current pose to the target_pose
	bool is_navigation_succeeded = move_base_navigation(goal->target_pose);

	// If the robot successfully reached the target pose, then detect obstacles
    assignment1::NavigationResult result;

    if (is_navigation_succeeded) {
        // Detect obstacles
        obstacles_detection(result);

        // Send the result and finish the action
        as.setSucceeded(result);
    } else {
        // Set the action as aborted
        as.setAborted();
    }
}

void NavigationServer::control_law_navigation(bool CL_flag) 
{
    if(CL_flag) {
        // Start the narrow corridor control law
        ROS_INFO("PROGRAM STATUS (SERVER): Robot has started moving through the narrow corridor with control_law");
        feedback.status = "Robot has started moving through the narrow corridor with control_law";
        as.publishFeedback(feedback);

        controller.control_law_navigation();

        ROS_INFO("PROGRAM STATUS (SERVER): Robot has passed the narrow corridor");
        feedback.status = "Robot has passed the narrow corridor";
        as.publishFeedback(feedback);
    }

    // Avoid control law to be activated for next clients
    this->CL_flag = false;

     ROS_INFO("PROGRAM STATUS (SERVER): Flag narrow corridor control law set to: [CL_flag=%d] (by server)", this->CL_flag);
}

bool NavigationServer::move_base_navigation(const geometry_msgs::Pose& target_pose) 
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
    string status_str = "[x=" + to_string(target_pose.position.x) + 
                        ", y=" + to_string(target_pose.position.y) + 
                        ", theta=" + to_string(theta) + "]";

    // Wait for move_base_client to be available
    move_base_client.waitForServer();

    // Send the goal target_pose to the move_base_client and wait for the result
    ROS_INFO("PROGRAM STATUS (SERVER): Robot has started moving towards the target pose with move_base: [x=%f, y=%f, theta=%f]", 
        target_pose.position.x, target_pose.position.y, theta);
    feedback.status = "Robot has started moving towards the target pose with move_base: " + status_str;
    as.publishFeedback(feedback);

    move_base_client.sendGoalAndWait(move_base_goal);

    // Chech whether the robot has successfully reached the target_pose
    if (move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    	// Robot has successfully reached the target_pose
        ROS_INFO("PROGRAM STATUS (SERVER): Robot has successfully reached the target pose");
        feedback.status = "Robot has successfully reached the target pose";
        as.publishFeedback(feedback);

		return true;
    } else {
    	// Robot has failed to reach the target_pose
        ROS_ERROR("PROGRAM STATUS (SERVER): Robot has failed to reach the target pose: invalid target pose (not in map)");
        feedback.status = "Robot has failed to reach the target pose: invalid target pose (not in map)";
        as.publishFeedback(feedback);

		return false;
    }
}

void NavigationServer::obstacles_detection(assignment1::NavigationResult& result) {
    // Obstacles detection using laser scan
    ROS_INFO("PROGRAM STATUS (SERVER): Robot has started detecting obstacles");
    feedback.status = "Robot has started detecting obstacles";
    as.publishFeedback(feedback);
    
    ObstaclesDetector obstacles_detector; 
    vector<CartesianPoint> detected_obstacles = obstacles_detector.detect_obstacle_positions();

    ROS_INFO("PROGRAM STATUS (SERVER): Robot has finished detecting obstacles");
    feedback.status = "Robot has finished detecting obstacles";
    as.publishFeedback(feedback);

    // Populate the feedback obstacle_position to send to the client
    for(const auto& obstacle : detected_obstacles) {
        geometry_msgs::Pose obstacle_position;
        obstacle_position.position.x = obstacle.x;
        obstacle_position.position.y = obstacle.y;

        result.obstacle_positions.push_back(obstacle_position);
    }
	
    // Print detected obstacle positions (no feedback to client, as required by assignment)
    ROS_INFO("PROGRAM STATUS (SERVER): Detected [%zu] obstacles", result.obstacle_positions.size());

    for (size_t i = 0; i < result.obstacle_positions.size(); i++) {
        ROS_INFO("ROBOT RESULT (SERVER): Obstacle [%zu]: [x=%f, y=%f]", 
            i+1, result.obstacle_positions[i].position.x, result.obstacle_positions[i].position.y);
    }
}
