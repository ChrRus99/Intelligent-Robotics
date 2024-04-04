#ifndef TIAGO_HEAD_CONTROLLER_H
#define TIAGO_HEAD_CONTROLLER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> HeadTrajectoryClient;

class TiagoHead
{
public:
	TiagoHead();
	~TiagoHead();

	/* This function moves the head to a given joint position */
	void move_head(double position);

private:
	HeadTrajectoryClient* head_trajectory_client;
};

#endif // TIAGO_HEAD_CONTROLLER_H