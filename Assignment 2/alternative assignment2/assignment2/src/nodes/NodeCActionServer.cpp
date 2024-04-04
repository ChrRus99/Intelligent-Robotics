#include "nodes/NodeCActionServer.h"
#include "util/Point.h"
#include "util/PointParser.h"
#include <custom_msgs/PickUpObject.h>
#include <custom_msgs/PickUpObstacle.h>
#include <string>

using namespace std;
using namespace Point;
using namespace PointParser;
using namespace ObjectsModels;

NodeCActionServer::NodeCActionServer()
	: as(nh, "object_manipulation_topic", boost::bind(&NodeCActionServer::manipulate_callback, this, _1), false)
{
	as.start();
}

void NodeCActionServer::manipulate_callback(const custom_msgs::ObjectManipulationGoalConstPtr& goal)
{
    
    // Send the goal target_pose to the move_base_client and wait for the result
    ROS_INFO("(NODE C SERVER): New client connected");

    bool is_manipulation_succeeded;

    if(goal->modality) {
        // If the goal is in pick-up modality
        custom_msgs::PickUpObject target_object_msg = goal->target_object;
        PickUpObject target_object = extract_pick_up_object_from_msg(target_object_msg);
        
        ROS_INFO("(NODE C SERVER): Robot arm has started the picking-up routine of the target object: %s", to_string(target_object));
        feedback.status = string("Robot arm has started the picking-up routine of the target object: ") + to_string(target_object);
        as.publishFeedback(feedback);

        // Extract data from the goal msgs        
        vector<custom_msgs::PickUpObject> objects_msgs = goal->objects;
        vector<custom_msgs::PickUpObstacle> obstacles_msgs = goal->obstacles;

        vector<PickUpObject> detected_objects;
        vector<PickUpObstacle> detected_obstacles;

        for(const auto& msg : objects_msgs) {
            detected_objects.push_back(extract_pick_up_object_from_msg(msg));
        }

        for(const auto& msg : obstacles_msgs) {
            detected_obstacles.push_back(extract_pick_up_obstacle_from_msg(msg));
        }

	    // Execute the picking-up routine of target object
        ObjectsManipulator manipulator(target_object, detected_objects, detected_obstacles);
	    is_manipulation_succeeded = manipulator.pick_up_target_object();
    } else {
        // If the goal is in place modality
        custom_msgs::PickUpObject target_object_msg = goal->target_object;
        PickUpObject target_object = extract_pick_up_object_from_msg(target_object_msg);

        ROS_INFO("(NODE C SERVER): Robot arm has started the placing routine of the target object: %s", to_string(target_object));
        feedback.status = string("Robot arm has started the placing routine of the target object: ") + to_string(target_object);
        as.publishFeedback(feedback);

        // Execute the placing routine of target object
        ObjectsManipulator manipulator(target_object);
	    is_manipulation_succeeded = manipulator.place_target_object();
    }

    // Send the result to the NodeC action client and finish the action
    custom_msgs::ObjectManipulationResult result;

    if (is_manipulation_succeeded) {
        ROS_INFO("(NODE C SERVER): Robot arm operation terminated successfully");
        feedback.status = "Robot arm operation terminated successfully";
        as.publishFeedback(feedback);

        result.is_succeeded = true;
        as.setSucceeded(result);
    } else {
        ROS_INFO("(NODE C SERVER): Robot arm operation failed");
        feedback.status = "Robot arm operation failed";
        as.publishFeedback(feedback);

        // Set the action as aborted
        //as.setAborted();

        result.is_succeeded = false;
        as.setSucceeded(result);
    }
}