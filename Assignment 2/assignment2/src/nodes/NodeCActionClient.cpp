#include "nodes/NodeCActionClient.h"
#include "util/PointParser.h"
#include <assignment2/PickUpObject.h>
#include <assignment2/PickUpObstacle.h>

using namespace std;
using namespace PointParser;
using namespace ObjectsModels;

NodeCActionClient::NodeCActionClient() : ac("object_manipulation_topic", true)
{
	// Wait for the action server to start
    ac.waitForServer();
}

void NodeCActionClient::send_pick_up_goal(const PickUpObject& target_object, 
                                          const vector<PickUpObject>& detected_objects, 
                                          const vector<PickUpObstacle>& detected_obstacles)
{
    // Build the messages with the detected objects and obstacles 
    vector<assignment2::PickUpObject> pick_up_objects_msgs;
    vector<assignment2::PickUpObstacle> pick_up_obstacles_msgs;

    for(const auto& object : detected_objects) {
        assignment2::PickUpObject object_msg = build_pick_up_object_msg_pose(object);
        pick_up_objects_msgs.push_back(object_msg);
    }

    for(const auto& obstacle : detected_obstacles) {
        assignment2::PickUpObstacle obstacle_msg = build_pick_up_obstacle_msg_pose(obstacle);
        pick_up_obstacles_msgs.push_back(obstacle_msg);
    }
    
    // Build the goal
    assignment2::ObjectManipulationGoal goal;
    goal.modality = 1;  // Pick-up modality
    goal.target_object = build_pick_up_object_msg_pose(target_object);
    goal.objects = pick_up_objects_msgs;
    goal.obstacles = pick_up_obstacles_msgs;

    // Send the goal
    ac.sendGoal(goal, 
        boost::bind(&NodeCActionClient::done_callback, this, _1, _2),
    	boost::bind(&NodeCActionClient::active_callback, this),
        boost::bind(&NodeCActionClient::feedback_callback, this, _1)
    );

    // Initialize the manipulation flag as in progress
    is_manipulation_finished = false;
}

void NodeCActionClient::send_delivery_goal(const PickUpObject& target_object)
{
    // Build the goal
    assignment2::ObjectManipulationGoal goal;
    goal.modality = 0;  // Deliver modality
    goal.target_object = build_pick_up_object_msg_pose(target_object);

    // Send the goal
    ac.sendGoal(goal, 
        boost::bind(&NodeCActionClient::done_callback, this, _1, _2),
    	boost::bind(&NodeCActionClient::active_callback, this),
        boost::bind(&NodeCActionClient::feedback_callback, this, _1)
    );

    // Initialize the manipulation flag as in progress
    is_manipulation_finished = false;
}

void NodeCActionClient::done_callback(const actionlib::SimpleClientGoalState& state, const assignment2::ObjectManipulationResultConstPtr& result) 
{
	// Process the result received from the server
    if(result->is_succeeded) {
        ROS_INFO("(NODE C CLIENT): Object manipulation successfully completed");
    } else {
        ROS_ERROR("(NODE C CLIENT): Object manipulation failed");
    }

    // Set the manipulation flag as terminated
    is_manipulation_finished = true;

    // Terminate the ROS node and exit the program
    ROS_INFO("(NODE C CLIENT): Client process is terminated");
    //ros::shutdown();
    //exit(0);
}

void NodeCActionClient::active_callback() 
{
	ROS_INFO("(NODE C CLIENT): Activated object manipulation of target_object");
}

void NodeCActionClient::feedback_callback(const assignment2::ObjectManipulationFeedbackConstPtr& feedback)
{
	// Process feedback received from the server
    ROS_INFO("(NODE C CLIENT): Feedback received: %s", feedback->status.c_str());
}