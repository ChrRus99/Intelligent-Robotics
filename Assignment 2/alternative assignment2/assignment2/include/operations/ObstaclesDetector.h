#ifndef OBSTACLES_DETECTOR_H
#define OBSTACLES_DETECTOR_H

#include "util/Point.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <move_base_msgs/MoveBaseAction.h>

class ObstaclesDetector
{
public:
    ObstaclesDetector();

    /* PUBLIC PROGRAM FUNCTIONS */
    std::vector<Point::CartesianPoint> detect_obstacle_positions();  
    
private:
    ros::NodeHandle nh; 
};

#endif // OBSTACLES_DETECTOR_H
