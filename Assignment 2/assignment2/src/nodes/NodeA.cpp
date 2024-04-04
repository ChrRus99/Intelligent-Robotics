#include "nodes/NodeA.h"
#include "util/PointParser.h"
#include "operations/DeliveryTablesDetector.h"
#include "nodes/NavigationActionClient.h"
#include "nodes/NodeBServiceServer.h"
#include "nodes/NodeCActionClient.h"
#include <tiago_iaslab_simulation/Objs.h> 
#include <assignment2/detected_objects.h>
#include <assignment2/PickUpObject.h>
#include <assignment2/PickUpObstacle.h>

using namespace std;
using namespace Point;
using namespace PointParser;
using namespace NavigationModels;
using namespace ObjectsModels;

NodeA::NodeA(bool extra_points_flag) 
{
    // Set the extra points flag
    this->extra_points_flag = extra_points_flag;
}


/* PUBLIC PROGRAM FUNCTIONS */
void NodeA::execute_pick_up_delivery_pipeline()
{
    ROS_INFO("(NODE A): Extra points routine activation status: %d", extra_points_flag);

    // Execute the pick-up & delivery pipeline of the program
    prepare_queue_of_objects_to_pick_up();
    build_navigation_trajectory();
    execute_navigation_trajectory();

    // Shutdown this node
    shutdown_node();
}


/* PRIVATE PROGRAM FUNCTIONS */ 
void NodeA::prepare_queue_of_objects_to_pick_up()
{
    ROS_INFO("(NODE A): Started preparation of pick-up objects queue");

    // Get the order in which to pick-up the objects from the human_node service 
    vector<int> object_ids = send_request_to_human_node_service_server();

    // Store the objects to pick-up in a queue in the order defined by the human_node service
    for(const auto& object_id : object_ids) {
        PickUpObject object(object_id);
        pick_up_objects_queue.push(object);
    }
}

void NodeA::build_navigation_trajectory()
{
    ROS_INFO("(NODE A): Started planning of full navigation trajectory for pick-up & delivery objects routine");

    bool was_root_created = false;
    TreeNode<NavigationPoseProperties>* pick_up_area_node;

    // For each object, build the (forward and backward) pick-up & delivery trajectory
    while(!pick_up_objects_queue.empty()) {
        // Extract the current target_object to pick-up 
        PickUpObject target_object = pick_up_objects_queue.front();
        pick_up_objects_queue.pop();   // consume the object from the queue

        // Get the pick-up table side position from which picking-up the current target object
        Position side_pick_up_table_position(0.0, 0.0);

        switch(target_object.get_object_type()) {
            case  PickUpObject::ObjectType::GREEN_TRIANGLE: {
                side_pick_up_table_position = FixedNavigationWayPoint::PICK_UP_TABLE_LEFT_POSITION;
                break;
            }
            case PickUpObject::ObjectType::BLUE_HEXAGON:
            case PickUpObject::ObjectType::RED_CUBE: {
                side_pick_up_table_position = FixedNavigationWayPoint::PICK_UP_TABLE_RIGHT_POSITION;
                break;
            }
        }

        // Compute the pose for picking-up the selected target_object from the pick-up table
        Pose pick_up_object_pose = get_navigation_pick_up_pose(target_object);

        // Compute the pose for delivering the selected target_object above the corresponding delivery table
        Pose delivery_pose = get_navigation_delivery_pose(target_object.get_object_type());


        /* BUILD TREE */
        if(!was_root_created) {
            /* (TREE MAIN BRANCH) NAVIGATION TO THE AREA OF THE PICK_UP TABLE */
            // Navigation from the starting pose to the pose of the pick-up table area
            TreeNode<NavigationPoseProperties>* root = navigation_trajectory_tree.add_root(
                {FixedNavigationWayPoint::START_POSE, NavigationPoseType::CROSSING_POSE}
            );
            pick_up_area_node = navigation_trajectory_tree.add_node(
                {FixedNavigationWayPoint::PICK_UP_AREA_POSITION, side_pick_up_table_position, NavigationPoseType::CROSSING_POSE}, 
                root
            );

            was_root_created = true;
        }
        
        /* (SUBTREE ROOT NODE) FORWARD NAVIGATION TO THE POSE ON ONE SIDE OF THE PICK_UP TABLE */
        // Navigation from the pose in the pick-up area to the pose on one side of the pick-up table
        TreeNode<NavigationPoseProperties>* side_pick_up_table_node = navigation_trajectory_tree.add_node(
            {side_pick_up_table_position, to_position(pick_up_object_pose), NavigationPoseType::CROSSING_POSE},
            pick_up_area_node
        );

        /* (BRANCH 1) FORWARD NAVIGATION TO THE POSE FOR PICKING-UP THE SELECTED OBJECT */
        // Navigation from the pose on one side of the pick-up table to the pose for picking-up the selected target_object
        TreeNode<NavigationPoseProperties>* pick_up_object_node = navigation_trajectory_tree.add_node(
            {pick_up_object_pose, NavigationPoseType::PICK_UP_OPERATIONS_POSE, target_object},
            side_pick_up_table_node
        );

        /* (BRANCH 2) BACKWARD NAVIGATION TO THE POSE ON THE LEFT SIDE OF THE PICK_UP TABLE */
        Pose pick_up_table_left_pose = to_pose(FixedNavigationWayPoint::PICK_UP_TABLE_LEFT_POSITION, 0.0);
        TreeNode<NavigationPoseProperties>* left_side_pick_up_table_node;

        // Change robot orientation towards the next point to reach, to avoid crashing against the table
        TreeNode<NavigationPoseProperties>* rotated_pick_up_object_node = navigation_trajectory_tree.add_node(
            {to_position(pick_up_object_pose), side_pick_up_table_position, NavigationPoseType::CROSSING_POSE},
            pick_up_object_node
        );

        if(side_pick_up_table_position == FixedNavigationWayPoint::PICK_UP_TABLE_RIGHT_POSITION) {
            // Navigation from the pose for picking-up the target object to the pose on the right side of the pick-up table
            TreeNode<NavigationPoseProperties>* right_side_pick_up_table_node = navigation_trajectory_tree.add_node(
                {side_pick_up_table_position, FixedNavigationWayPoint::PICK_UP_TABLE_LEFT_POSITION, NavigationPoseType::CROSSING_POSE},
                rotated_pick_up_object_node
            );
            // Navigation from the right side of the pick-up table to the left side of the pick-up table
            left_side_pick_up_table_node = navigation_trajectory_tree.add_node(
                {pick_up_table_left_pose, NavigationPoseType::CROSSING_POSE},
                right_side_pick_up_table_node
            );
        } else {
            // Navigation from the pose for picking-up the target object to the pose on the left side of the pick-up table
            left_side_pick_up_table_node = navigation_trajectory_tree.add_node(
                {pick_up_table_left_pose, NavigationPoseType::CROSSING_POSE},
                rotated_pick_up_object_node
            );
        }
  
        /* (BRANCH 2) FORWARD NAVIGATION TO THE POSE FOR DELIVERING THE SELECTED OBJECT */
        // Navigation from the pose on the left side of the pick-up table to the pose in the delivery area
        TreeNode<NavigationPoseProperties>* delivery_area_node = navigation_trajectory_tree.add_node(
            {FixedNavigationWayPoint::DELIVERY_AREA_POSE, NavigationPoseType::CROSSING_POSE},
            left_side_pick_up_table_node
        );

        // Navigation from the pose the delivery area to the pose in front of the delivery table of the target_object 
        TreeNode<NavigationPoseProperties>* delivery_operations_node = navigation_trajectory_tree.add_node(
            {delivery_pose, NavigationPoseType::DELIVERY_OPERATIONS_POSE, target_object, extra_points_flag},
            delivery_area_node
        );
    }
}

void NodeA::execute_navigation_trajectory()
{
    ROS_INFO("(NODE A): Started execution of navigation trajectory for pick-up & delivery of objects");

    depth_first_search(navigation_trajectory_tree.get_root(), &callback_poses_operations);
}

void NodeA::pick_up_object_from_pick_up_table(const PickUpObject& target_object)
{
    /* DETECTION OF OBJECTS AND OBSTACLES ON PICK-UP TABLE */
    ROS_INFO("(NODE A): Started detection procedure of objects on pick-up table");

    // Get the pick-up objects and the pick-up objstacles from the NodeB publisher 
    // Important node: if no message is received, the robot stops running and waits until it receives a message from NodeB!
    pair<vector<PickUpObject>, vector<PickUpObstacle>> result = send_request_to_node_B_service_server();
    vector<PickUpObject> detected_objects = result.first;
    vector<PickUpObstacle> detected_obstacles = result.second;

    ROS_INFO("(NODE A): Detected [%ld] objects on pick-up table:", detected_objects.size());
    for(const auto& object : detected_objects) {
        ROS_INFO("(NODE A): %s", to_string(object));
    }

    ROS_INFO("(NODE A): Detected [%ld] obstacles on pick-up table:", detected_obstacles.size());
    for(const auto& obstacle : detected_obstacles) {
        ROS_INFO("(NODE A): %s", to_string(obstacle));
    }

    /* PICK-UP PROCEDURE OF TARGET OBJECT FROM PICK-UP TABLE */
    ROS_INFO("(NODE A): Started pick-up procedure from pick-up table of target_object: %s", to_string(target_object));

    // Send the goal to the nodeC action server for picking-up the target object
    call_node_C_action_server_to_pick_up_object(target_object, detected_objects, detected_obstacles);
}

void NodeA::place_object_on_delivery_table(const PickUpObject& target_object)
{
    /* DELIVERY PROCEDURE OF TARGET OBJECT ON DELIVERY TABLE */
    ROS_INFO("(NODE A): Started delivery procedure of target_object: %s on delivery table: %s", 
    to_string(target_object), to_string(get_corresponding_delivery_table_type(target_object.get_object_type())));

    // Send the goal to the nodeC action server for placing the target object
    call_node_C_action_server_to_place_object(target_object);
}

Pose NodeA::extra_points_procedure(const PickUpObject& target_object) 
{
    ROS_INFO("(NODE A): Robot has started detecting delivery tables");

    DeliveryTablesDetector detector(FixedNavigationWayPoint::DELIVERY_AREA_POSE);
    DeliveryTable target_delivery_table = detector.get_target_delivery_table(target_object);
    Pose navigation_delivery_pose = target_delivery_table.get_pose();

    ROS_INFO("(NODE A): Robot has identified the target delibery table: %s", to_string(target_delivery_table));

    return navigation_delivery_pose;
}

void NodeA::shutdown_node()
{
    ROS_INFO("(NODE A): Press a key to shutdown the Node A");
    getchar();
    ROS_INFO("(NODE A): Node A shutdown");
    ros::shutdown();
    exit(0);
}


/* PRIVATE REQUESTS FUNCTIONS */
vector<int> NodeA::send_request_to_human_node_service_server()
{	
	// Create a service client for the human_node service
	ros::ServiceClient client = nh.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");

	// Create a service request
    tiago_iaslab_simulation::Objs srv;

    // Set the request to ready for receiving the object's ID 
    srv.request.ready = true;
    srv.request.all_objs = true;

    // Extract the sequence of objects' IDs to pick up
    vector<int> object_ids;

    ROS_INFO("(NODE A): Send request to human_node service server");

	// Call the service (and wait for response)
	if(client.call(srv)) {
        ROS_INFO("(NODE A): Received response from human_node: Number of objects' IDs: %ld", srv.response.ids.size());

        for (int i=0; i<srv.response.ids.size(); i++) {
            object_ids.push_back(srv.response.ids.at(i));
            ROS_INFO("(NODE A): Object number: %d, has ID: %d", i, srv.response.ids.at(i));
        }
    } else {
        ROS_ERROR("(NODE A): Failed to call human_node service. Shutting down NodeA.");
        ros::shutdown(); // Terminate the node
    }

    return object_ids;
}

void NodeA::call_navigation_action_server(const Pose& target_pose)
{
    ROS_INFO("(NODE A): Call navigation action server (using navigation action client)");

    // Use the navigation action client to send the goal to the navigation action server
    NavigationActionClient client;
    client.send_goal(target_pose);

    // Wait the navigation to terminate before continue
    ros::Rate loop_rate(1);  // 1 Hz
    
    while (!client.check_navigation_finished() && ros::ok()) {
        // Allow ROS to process callbacks
        ros::spinOnce();
        loop_rate.sleep();
    }
}

pair<vector<PickUpObject>, vector<PickUpObstacle>> NodeA::send_request_to_node_B_service_server()
{
    // Create a service client for the human_node service
    ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<assignment2::detected_objects>("/detected_objects_topic");

	// Create a service request
    assignment2::detected_objects srv;

    // Set the request to ready for receiving the object's ID 
    srv.request.is_ready = true;

    // Extract 
    vector<assignment2::PickUpObject> objects_msgs;
    vector<assignment2::PickUpObstacle> obstacles_msgs;

    ROS_INFO("(NODE A): Sent request to Node B service server");

	// Call the service (and wait for response)
	if(client.call(srv)) {
        ROS_INFO("(NODE A): Received response from Node B:");
        ROS_INFO("(NODE A): Number of detected objects: [%ld]", srv.response.objects.size());
        ROS_INFO("(NODE A): Number of detected obstacles: [%ld]", srv.response.obstacles.size());

        objects_msgs = srv.response.objects;
        obstacles_msgs = srv.response.obstacles;
    } else {
        ROS_ERROR("(NODE A): Failed to call Node B service. Shutting down NodeA.");
        ros::shutdown(); // Terminate the node
    }

    // Transform the vector of pick_up_object msgs into a vector of PickUpObject
    vector<PickUpObject> objects;
    for(const auto& object_msg : objects_msgs) {
        PickUpObject object = extract_pick_up_object_from_msg(object_msg);
        
        ROS_INFO("(NODE A): Detected object: %s", to_string(object));

        objects.push_back(object);
    }

    // Transform the vector of pick_up_obstacles msgs into a vector of PickUpObstacle
    vector<PickUpObstacle> obstacles;
    for(const auto& obstacle_msg : obstacles_msgs) {
        PickUpObstacle obstacle = extract_pick_up_obstacle_from_msg(obstacle_msg);
        
        ROS_INFO("(NODE A): Detected obstacle: %s", to_string(obstacle));

        obstacles.push_back(obstacle);
    }

    return pair<vector<PickUpObject>, vector<PickUpObstacle>>(objects, obstacles);
}

void NodeA::call_node_C_action_server_to_pick_up_object(const PickUpObject& target_object, 
                                                        const vector<PickUpObject>& detected_objects, 
                                                        const vector<PickUpObstacle>& detected_obstacles)
{
    ROS_INFO("(NODE A): Call node C action server (using node C action client)for picking-up the target object: %s", to_string(target_object));

    // Use the nodeC action client to send the pick-up goal to the nodeC action server
    NodeCActionClient client;
    client.send_pick_up_goal(target_object, detected_objects, detected_obstacles);

    // Wait the navigation to terminate before continue
    ros::Rate loop_rate(1);  // 1 Hz
    
    while (!client.check_manipulation_finished() && ros::ok()) {
        // Allow ROS to process callbacks
        ros::spinOnce();  
        loop_rate.sleep();
    }
}

void NodeA::call_node_C_action_server_to_place_object(const PickUpObject& target_object)
{
    ROS_INFO("(NODE A): Call node C action server (using node C action client) for placing the object: %s", to_string(target_object));

    // Use the nodeC action client to send the delivery goal to the nodeC action server
    NodeCActionClient client;
    client.send_delivery_goal(target_object);

    // Wait the navigation to terminate before continue
    ros::Rate loop_rate(1);  // 1 Hz
    
    while (!client.check_manipulation_finished() && ros::ok()) {
        // Allow ROS to process callbacks
        ros::spinOnce();  
        loop_rate.sleep();
    }
}


/* PRIVATE CALLBACK FUNCTIONS */
void NodeA::callback_poses_operations(NavigationPoseProperties& pose_properties)
{
    Pose pose = pose_properties.pose;
    NavigationPoseType pose_type = pose_properties.pose_type;
    PickUpObject target_object = pose_properties.target_object;

    // Execute operations in target_pose
    switch(pose_type) {
        case NavigationPoseType::CROSSING_POSE: {
            // Navigate to target_pose
            NodeA::call_navigation_action_server(pose);
            ROS_INFO("(NODE A): Current pose is a [CROSSING_POSE]: %s", to_string(pose));
            break;
        }
        case NavigationPoseType::PICK_UP_OPERATIONS_POSE: {
            // Navigate to target_pose
            NodeA::call_navigation_action_server(pose);
            ROS_INFO("(NODE A): Current pose is a [PICK_UP_OPERATIONS_POSE]: %s", to_string(pose));
            
            // Execute pick-up routine
            pick_up_object_from_pick_up_table(target_object);
            break;
        }
        case NavigationPoseType::DELIVERY_OPERATIONS_POSE: {
            if(pose_properties.extra_points_flag) {
                // Execute extra points procedure to obtain the delivery table target_pose
                pose = extra_points_procedure(target_object);
            }

            // Navigate to target_pose
            NodeA::call_navigation_action_server(pose);
            ROS_INFO("(NODE A): Current pose is a [DELIVERY_OPERATIONS_POSE]: %s", to_string(pose));

            // Execute the delivery routine
            place_object_on_delivery_table(target_object);
            break;
        }
    }
}