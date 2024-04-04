#include "util/PointParser.h"
#include <cmath>

using namespace std;
using namespace Point;
using namespace ObjectsModels;

namespace PointParser {
    geometry_msgs::Pose build_geometry_msgs_pose(const Point::Pose& pose)
    {
        geometry_msgs::Pose target_pose;

        // Calculate the z and w components of the orientation quaternion
        double theta = pose.theta;
        theta = theta * (M_PI / 180.0); // degree to rad
        double z = sin(theta / 2.0);
        double w = cos(theta / 2.0);

        // Populate target_pose with the input data
        target_pose.position.x = pose.x;
        target_pose.position.y = pose.y;
        target_pose.position.z = 0.0;
        target_pose.orientation.x = 0.0;
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = z;
        target_pose.orientation.w = w;

        return target_pose;
    }

    PickUpObject extract_pick_up_object_from_msg(const custom_msgs::PickUpObject& object_msg)
    {
        // Extract data from the message and build a PickUpObject
        Position object_position(
            object_msg.object_position.position.x, 
            object_msg.object_position.position.y, 
            object_msg.object_position.position.z
        );
        PickUpObject object(object_msg.object_id, object_position);

        return object;
    }

    PickUpObstacle extract_pick_up_obstacle_from_msg(const custom_msgs::PickUpObstacle& obstacle_msg)
    {
        // Extract data from the message and build a PickUpObstacle
        Position obstacle_position(
            obstacle_msg.obstacle_position.position.x, 
            obstacle_msg.obstacle_position.position.y, 
            obstacle_msg.obstacle_position.position.z
        );
        PickUpObstacle obstacle(obstacle_msg.obstacle_id, obstacle_position);

        return obstacle;
    }

    custom_msgs::PickUpObject build_pick_up_object_msg_pose(const PickUpObject& object)
    {
        custom_msgs::PickUpObject object_msg;

        // Populate pick_up_object msg with the pick_up_object data
        object_msg.object_id = object.get_object_id();
        object_msg.object_position.position.x = object.get_object_position().x;
        object_msg.object_position.position.y = object.get_object_position().y;
		object_msg.object_position.position.z = object.get_object_position().z;

        return object_msg;
    }
    
    custom_msgs::PickUpObstacle build_pick_up_obstacle_msg_pose(const PickUpObstacle& obstacle)
    {
        custom_msgs::PickUpObstacle obstacle_msg;

        // Populate pick_up_obstacle msg with the pick_up_obstacle data
        obstacle_msg.obstacle_id = obstacle.get_obstacle_id();
        obstacle_msg.obstacle_position.position.x = obstacle.get_obstacle_position().x;
        obstacle_msg.obstacle_position.position.y = obstacle.get_obstacle_position().y;
        obstacle_msg.obstacle_position.position.z = obstacle.get_obstacle_position().z; 

        return obstacle_msg;
    }
}
