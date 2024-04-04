#ifndef NODE_A_H
#define NODE_A_H

#include "util/Point.h"
#include "util/Tree.h"
#include "models/ObjectsModels.h"
#include "models/NavigationModels.h"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <queue>

class NodeA
{
public:
    NodeA(bool extra_points_flag);

    /* PUBLIC PROGRAM FUNCTIONS */
    /* This function executes the pick-up & delivery pipeline of the program */
    void execute_pick_up_delivery_pipeline();

private:
    ros::NodeHandle nh;
    std::queue<ObjectsModels::PickUpObject> pick_up_objects_queue;
    Tree<NavigationModels::NavigationPoseProperties> navigation_trajectory_tree;
    bool extra_points_flag;

    /* PRIVATE PROGRAM FUNCTIONS */ 
    /* This function defines a queue of objects to pick-up the objects in the order defined by the human_node service */
    void prepare_queue_of_objects_to_pick_up();
    /* This function builds the full trajectory for the full pick-up & delivery pipeline of the program */
    void build_navigation_trajectory();
    /* This function executes the navigation along the full trajectory for the full pick-up & delivery pipeline of the program */
    void execute_navigation_trajectory();
    /* This function makes the robot pick-up the object from them pick-up table */
    static void pick_up_object_from_pick_up_table(const ObjectsModels::PickUpObject& target_object);
    /* This function makes the robot place the object to the delivery table */
    static void place_object_on_delivery_table(const ObjectsModels::PickUpObject& target_object);
    /* This function exectutes the extra points part procedure */
    static Point::Pose extra_points_procedure(const ObjectsModels::PickUpObject& target_object);
    /* This function allows the node A to be shutdown */
    void shutdown_node();

    /* PRIVATE REQUESTS FUNCTIONS */
    /* This function sends a request to the human_node service server to get the ID of the objects to pick-up */ 
    std::vector<int> send_request_to_human_node_service_server();
    /* This function calls the navigation action server to make the robot reach the target_pose */
    static void call_navigation_action_server(const Point::Pose& target_pose);
    /* This function subscribes to the NodeB to get the positions of the objects and of the obstacles on the pick-up table */
    static std::pair<std::vector<ObjectsModels::PickUpObject>, std::vector<ObjectsModels::PickUpObstacle>> send_request_to_node_B_service_server();
    /* This function calls the NodeC action server to make the robot pick-up the given object from the pick-up table */
    static void call_node_C_action_server_to_pick_up_object(const ObjectsModels::PickUpObject& target_object, 
                                                            const std::vector<ObjectsModels::PickUpObject>& detected_objects, 
                                                            const std::vector<ObjectsModels::PickUpObstacle>& detected_obstacles);
    /* This function calls the NodeC action server to make the robot place the picked-up object on the corresponding delivery table */
    static void call_node_C_action_server_to_place_object(const ObjectsModels::PickUpObject& target_object);

    /* PRIVATE CALLBACK FUNCTIONS */
    /* Function to be passed as a callback to depth_first_search */
    static void callback_poses_operations(NavigationModels::NavigationPoseProperties& pose_properties);
};

/* HELPER FUNCTIONS */


#endif // NODE_A_H
