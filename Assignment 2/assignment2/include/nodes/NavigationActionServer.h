#ifndef NAVIGATION_ACTION_SERVER_H
#define NAVIGATION_ACTION_SERVER_H

#include "operations/ObstaclesDetector.h"
#include <ros/ros.h>
#include <assignment2/NavigationAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

class NavigationActionServer
{
public:
    NavigationActionServer(); 
    
private:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<assignment2::NavigationAction> as;
    assignment2::NavigationFeedback feedback;
    assignment2::NavigationResult result;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;
    ObstaclesDetector obstacles_detector;
    
    /* CALLBACK FUNCTIONS */
    void navigate_callback(const assignment2::NavigationGoalConstPtr &goal);

    /* PRIVATE PROGRAM FUNCTIONS */
    bool move_base_navigation(const geometry_msgs::Pose& target_pose);
    void obstacles_detection(assignment2::NavigationResult& result);
};

#endif // NAVIGATION_ACTION_SERVER_H