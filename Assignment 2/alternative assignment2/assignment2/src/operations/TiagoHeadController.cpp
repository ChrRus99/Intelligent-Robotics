#include "operations/TiagoHeadController.h"

TiagoHead::TiagoHead()
{
    // Initialize the client for the Action interface to the head trajectory controller
	head_trajectory_client = new HeadTrajectoryClient("/head_controller/follow_joint_trajectory", true);

	// Wait for head trajectory controller action server to come up 
	while (!head_trajectory_client->waitForServer(ros::Duration(5.0))) {
	    ROS_INFO("(TIAGO HEAD CONTROLLER): Waiting for the head trajectory controller action server to come up");
    }
}

TiagoHead::~TiagoHead()
{
    delete head_trajectory_client;
}

void TiagoHead::move_head(double position)
{
    ROS_INFO("(TIAGO HEAD CONTROLLER): Head movement started");
	// The goal message
	control_msgs::FollowJointTrajectoryGoal goal;

	// Set the joint names for the head joints (we are forced to specify both of them)
	goal.trajectory.joint_names.push_back("head_1_joint");
	goal.trajectory.joint_names.push_back("head_2_joint");

	// Create a trajectory point
	trajectory_msgs::JointTrajectoryPoint point;
	point.positions.push_back(0);  // Position for head_1_joint
	point.positions.push_back(position);  // Position for head_2_joint
	point.time_from_start = ros::Duration(1.0);

	// Add the trajectory point to the goal
	goal.trajectory.points.push_back(point);

	// Send the goal
	head_trajectory_client->sendGoal(goal);

	// Wait for it to reach the goal (abort after 2 seconds to prevent getting stuck)
	head_trajectory_client->waitForResult(ros::Duration(5));
	
	ROS_INFO("(TIAGO HEAD CONTROLLER): Head movement finished");
}