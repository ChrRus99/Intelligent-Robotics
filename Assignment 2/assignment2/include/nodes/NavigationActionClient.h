#ifndef NAVIGATION_ACTION_CLIENT_H
#define NAVIGATION_ACTION_CLIENT_H

#include "util/Point.h"
#include "util/PointParser.h"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment2/NavigationAction.h>

class NavigationActionClient
{
public:
    NavigationActionClient();

    /* PUBLIC PROGRAM FUNCTIONS */
    void send_goal(const Point::Pose& target_pose);

    inline bool check_navigation_finished() { return is_navigation_finished; }
    
private:
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<assignment2::NavigationAction> ac; 
    bool is_navigation_finished; 

    /* CALLBACK FUNCTIONS */  
    void done_callback(const actionlib::SimpleClientGoalState& state,
                      const assignment2::NavigationResultConstPtr& result);
	void active_callback();
	void feedback_callback(const assignment2::NavigationFeedbackConstPtr& feedback);

    /* PRIVATE PROGRAM FUNCTIONS */
    
};

#endif // NAVIGATION_ACTION_CLIENT_H
