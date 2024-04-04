#ifndef OBJECTS_DETECTOR_H
#define OBJECTS_DETECTOR_H

#include "util/Point.h"
#include "models/ObjectsModels.h"
#include "operations/TiagoHeadController.h"
#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

class ObjectsDetector
{
public:
    ObjectsDetector();

    /* PUBLIC PROGRAM FUNCTIONS */
    /* This functions returns the objects detected on the pick-up table */
    inline std::vector<ObjectsModels::PickUpObject> get_detected_objects() {return objects;}
    /* This functions the the obstacles detected on the pick-up table */
    inline std::vector<ObjectsModels::PickUpObstacle> get_detected_obstacles() {return obstacles;}
    
private:
    ros::NodeHandle nh; 
    ros::Subscriber apriltag_sub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    TiagoHead tiago_head;
    
    std::vector<ObjectsModels::PickUpObject> objects;
    std::vector<ObjectsModels::PickUpObstacle> obstacles;

    /* PRIVATE PROGRAM FUNCTIONS */
    void detect_objects_on_pick_up_table();
};

#endif // OBJECTS_DETECTOR_H
