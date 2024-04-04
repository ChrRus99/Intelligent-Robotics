#ifndef NAVIGATION_SERVER_H
#define NAVIGATION_SERVER_H

#include <ros/ros.h>
#include <assignment1/NavigationAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "ObstaclesDetector.h"
#include "NarrowCorridorControlLaw.h"

class NavigationServer
{
public:
    NavigationServer(bool CL_flag=false); 
    
private:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<assignment1::NavigationAction> as;
    assignment1::NavigationFeedback feedback;
    assignment1::NavigationResult result;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;
    ObstaclesDetector obstacles_detector;
    NarrowCorridorControlLaw controller;
    bool CL_flag;
    
    /* CALLBACK FUNCTIONS */
    void navigate_callback(const assignment1::NavigationGoalConstPtr &goal);

    /* PRIVATE PROGRAM FUNCTIONS */
    void control_law_navigation(bool CL_flag);
    bool move_base_navigation(const geometry_msgs::Pose& target_pose);
    void obstacles_detection(assignment1::NavigationResult& result);
};

#endif // NAVIGATION_SERVER_H
