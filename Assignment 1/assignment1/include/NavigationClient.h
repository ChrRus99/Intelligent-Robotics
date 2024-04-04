#ifndef NAVIGATION_CLIENT_H
#define NAVIGATION_CLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment1/NavigationAction.h>

class NavigationClient
{
public:
    NavigationClient();

    /* PUBLIC PROGRAM FUNCTIONS */
    void sendGoal(const geometry_msgs::Pose& target_pose);
    
private:
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<assignment1::NavigationAction> ac;  

    /* CALLBACK FUNCTIONS */  
    void doneCallback(const actionlib::SimpleClientGoalState& state,
                      const assignment1::NavigationResultConstPtr& result);
	void activeCallback();
	void feedbackCallback(const assignment1::NavigationFeedbackConstPtr& feedback);
};

#endif // NAVIGATION_CLIENT_H
