#ifndef UTIL_POINT_PARSER_H
#define UTIL_POINT_PARSER_H

#include "util/Point.h"
#include "models/ObjectsModels.h"
#include <geometry_msgs/Pose.h>
#include <assignment2/PickUpObject.h>
#include <assignment2/PickUpObstacle.h>
#include <vector>

namespace PointParser {
    /* HELPER FUNCTIONS */
    /* This function converts a pose to a geometry_msgs pose */
    geometry_msgs::Pose build_geometry_msgs_pose(const Point::Pose& pose);
    /* This function converts a pick_up_object_msg to a pick_up_object */
    ObjectsModels::PickUpObject extract_pick_up_object_from_msg(const assignment2::PickUpObject& object_msg);
    /* This function converts a pick_up_obstacle_msg to a pick_up_obstacle */
    ObjectsModels::PickUpObstacle extract_pick_up_obstacle_from_msg(const assignment2::PickUpObstacle& obstacle_msg);
    /* This function converts an object position to a pick_up_object msg pose */
    assignment2::PickUpObject build_pick_up_object_msg_pose(const ObjectsModels::PickUpObject& object);
    /* This function converts an obstacle position to a pick_up_obstacle msg pose */
    assignment2::PickUpObstacle build_pick_up_obstacle_msg_pose(const ObjectsModels::PickUpObstacle& obstacle);
}

#endif // UTIL_POINT_PARSER_H
