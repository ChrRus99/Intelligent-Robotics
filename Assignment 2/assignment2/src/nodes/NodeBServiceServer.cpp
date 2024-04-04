#include "nodes/NodeBServiceServer.h"
#include "util/Point.h"
#include "util/PointParser.h"
#include "models/ObjectsModels.h"

using namespace std;
using namespace Point;
using namespace PointParser;
using namespace ObjectsModels;

NodeBServiceServer::NodeBServiceServer() 
{
    service = nh.advertiseService("detected_objects_topic", &NodeBServiceServer::handle_detected_objects_service, this);
    ROS_INFO("(NODE B SERVER): Ready to handle detected_objects_service requests.");
}

bool NodeBServiceServer::handle_detected_objects_service(assignment2::detected_objects::Request& req,
                                                         assignment2::detected_objects::Response& res)
{
    ObjectsDetector detector;

    // Get the objects and the obstacles detected on the pick-up table
    vector<PickUpObject> detected_objects = detector.get_detected_objects();
    vector<PickUpObstacle> detected_obstacles = detector.get_detected_obstacles();

    ROS_INFO("(Node B SERVER): Number of detected [%ld] objects", detected_objects.size());
    ROS_INFO("(Node B SERVER): Number of detected [%ld] obstacles", detected_obstacles.size());

    // If the publisher has something to publish
    if(/*req.is_ready &&*/ !detected_objects.empty() || !detected_obstacles.empty()) {
        // Build the message with the detected objects and obstacles 
        vector<assignment2::PickUpObject> pick_up_objects_msgs;
        vector<assignment2::PickUpObstacle> pick_up_obstacles_msgs;

        for(const auto& object : detected_objects) {
            ROS_INFO("(Node B SERVER): Detected object: %s", to_string(object));

            assignment2::PickUpObject object_msg = build_pick_up_object_msg_pose(object);
            pick_up_objects_msgs.push_back(object_msg);
        }

        for(const auto& obstacle : detected_obstacles) {
            ROS_INFO("(Node B SERVER): Detected obstacle: %s", to_string(obstacle));

            assignment2::PickUpObstacle obstacle_msg = build_pick_up_obstacle_msg_pose(obstacle);
            pick_up_obstacles_msgs.push_back(obstacle_msg);
        }

        // Build the response with the previous messages
        res.header.stamp = ros::Time::now();
        res.objects = pick_up_objects_msgs;
        res.obstacles = pick_up_obstacles_msgs;

        ROS_INFO("(NODE B SERVER): Detected objects and obstacles sent to Node B service client");
    }

    return true;
}