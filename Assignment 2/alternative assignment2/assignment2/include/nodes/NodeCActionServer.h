#ifndef NODE_C_ACTION_SERVER_H
#define NODE_C_ACTION_SERVER_H

#include "operations/ObjectsManipulator.h"
#include <ros/ros.h>
#include <custom_msgs/ObjectManipulationAction.h>
#include <custom_msgs/PickUpObject.h>
#include <custom_msgs/PickUpObstacle.h>
#include <actionlib/server/simple_action_server.h>

class NodeCActionServer
{
public:
    NodeCActionServer(); 
    
private:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<custom_msgs::ObjectManipulationAction> as;
    custom_msgs::ObjectManipulationFeedback feedback;
    custom_msgs::ObjectManipulationResult result;
    
    /* CALLBACK FUNCTIONS */
    void manipulate_callback(const custom_msgs::ObjectManipulationGoalConstPtr& goal);

    /* PRIVATE PROGRAM FUNCTIONS */
    bool object_manipulation(const custom_msgs::ObjectManipulationGoalConstPtr& goal);
};

#endif // NODE_C_ACTION_ACTION_SERVER_H