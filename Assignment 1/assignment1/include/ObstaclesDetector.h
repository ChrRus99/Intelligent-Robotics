#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "util/Point.h"

class ObstaclesDetector
{
public:
    ObstaclesDetector();

    /* PUBLIC PROGRAM FUNCTIONS */
    std::vector<Point::CartesianPoint> detect_obstacle_positions();  
    
private:
    ros::NodeHandle nh; 
};

#endif // OBJECT_DETECTOR_H
