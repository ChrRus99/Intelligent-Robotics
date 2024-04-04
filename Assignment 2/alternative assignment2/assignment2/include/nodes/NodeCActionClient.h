#ifndef NODE_C_ACTION_CLIENT_H
#define NODE_C_ACTION_CLIENT_H

#include "models/ObjectsModels.h"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <custom_msgs/ObjectManipulationAction.h>

using namespace ObjectsModels;

class NodeCActionClient
{
public:
    NodeCActionClient();

    /* PUBLIC PROGRAM FUNCTIONS */
    void send_pick_up_goal(const PickUpObject& target_object, 
                           const std::vector<ObjectsModels::PickUpObject>& detected_objects, 
                           const std::vector<ObjectsModels::PickUpObstacle>& detected_obstacles);
    void send_delivery_goal(const PickUpObject& target_object);

    inline bool check_manipulation_finished() { return is_manipulation_finished; }
    
private:
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<custom_msgs::ObjectManipulationAction> ac; 
    bool is_manipulation_finished; 

    /* CALLBACK FUNCTIONS */  
    void done_callback(const actionlib::SimpleClientGoalState& state,
                      const custom_msgs::ObjectManipulationResultConstPtr& result);
	void active_callback();
	void feedback_callback(const custom_msgs::ObjectManipulationFeedbackConstPtr& feedback);

    /* PRIVATE PROGRAM FUNCTIONS */
    
};

#endif // NODE_C_ACTION_CLIENT_H
