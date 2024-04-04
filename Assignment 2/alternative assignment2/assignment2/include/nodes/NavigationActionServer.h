#ifndef NAVIGATION_ACTION_SERVER_H
#define NAVIGATION_ACTION_SERVER_H

#include "operations/ObstaclesDetector.h"
#include <ros/ros.h>
#include <custom_msgs/NavigationAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

class NavigationActionServer
{
public:
    NavigationActionServer(); 
    
private:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<custom_msgs::NavigationAction> as;
    custom_msgs::NavigationFeedback feedback;
    custom_msgs::NavigationResult result;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;
    ObstaclesDetector obstacles_detector;
    
    /* CALLBACK FUNCTIONS */
    void navigate_callback(const custom_msgs::NavigationGoalConstPtr &goal);

    /* PRIVATE PROGRAM FUNCTIONS */
    bool move_base_navigation(const geometry_msgs::Pose& target_pose);
    void obstacles_detection(custom_msgs::NavigationResult& result);
};

#endif // NAVIGATION_ACTION_SERVER_H