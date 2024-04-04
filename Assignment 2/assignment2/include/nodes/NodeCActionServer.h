#ifndef NODE_C_ACTION_SERVER_H
#define NODE_C_ACTION_SERVER_H

#include "operations/ObjectsManipulator.h"
#include <ros/ros.h>
#include <assignment2/ObjectManipulationAction.h>
#include <assignment2/PickUpObject.h>
#include <assignment2/PickUpObstacle.h>
#include <actionlib/server/simple_action_server.h>

class NodeCActionServer
{
public:
    NodeCActionServer(); 
    
private:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<assignment2::ObjectManipulationAction> as;
    assignment2::ObjectManipulationFeedback feedback;
    assignment2::ObjectManipulationResult result;
    
    /* CALLBACK FUNCTIONS */
    void manipulate_callback(const assignment2::ObjectManipulationGoalConstPtr& goal);

    /* PRIVATE PROGRAM FUNCTIONS */
    bool object_manipulation(const assignment2::ObjectManipulationGoalConstPtr& goal);
};

#endif // NODE_C_ACTION_ACTION_SERVER_H