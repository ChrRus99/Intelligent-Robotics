#ifndef NAVIGATION_MODELS_H
#define NAVIGATION_MODELS_H

#include "util/Point.h"
#include "models/ObjectsModels.h"

namespace NavigationModels {
    /* NAVIGATION_POSE_TYPE */
    enum class NavigationPoseType {
        CROSSING_POSE,
        PICK_UP_OPERATIONS_POSE,
        DELIVERY_OPERATIONS_POSE
    };


    /* NAVIGATION_POSE_PROPERTIES */
    struct NavigationPoseProperties
    {
        Point::Pose pose;
        NavigationPoseType pose_type;
        ObjectsModels::PickUpObject target_object;
        bool extra_points_flag;

        NavigationPoseProperties(Point::Pose pose, 
                                 NavigationPoseType pose_type, 
                                 ObjectsModels::PickUpObject target_object = ObjectsModels::PickUpObject(
                                    ObjectsModels::PickUpObject::ObjectType::NONE
                                 ),
                                 bool extra_points_flag = false
        ) : pose(pose), pose_type(pose_type), target_object(target_object), extra_points_flag(extra_points_flag) {}

        NavigationPoseProperties(Point::Position curr_position,
                                 Point::Position next_position, 
                                 NavigationPoseType pose_type, 
                                 ObjectsModels::PickUpObject target_object = ObjectsModels::PickUpObject(
                                    ObjectsModels::PickUpObject::ObjectType::NONE
                                 ),
                                 bool extra_points_flag = false
        ) : NavigationPoseProperties( 
            // Create a pose in curr_position, oriented towards the next_position to reach
            Point::Pose(curr_position.x, curr_position.y, angle(curr_position, next_position)),
            pose_type, 
            target_object,
            extra_points_flag
        ) {}
    };

    /* FIXED_NAVIGATION_WAY_POINT */
    class FixedNavigationWayPoint {
    public:
        // Delete the default constructor to make this class non-instantiable
        //FixedPoses() = delete; 

        /* Fixed poses declaration */
        static const Point::Pose START_POSE;
        static const Point::Position PICK_UP_AREA_POSITION;
        static const Point::Position PICK_UP_TABLE_LEFT_POSITION;
        static const Point::Position PICK_UP_TABLE_RIGHT_POSITION;
        static const Point::Pose DELIVERY_AREA_POSE;
    };
}

#endif // NAVIGATION_MODELS_H