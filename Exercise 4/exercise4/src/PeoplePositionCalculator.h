#ifndef PEOPLE_POSITION_CALCULATOR_H
#define PEOPLE_POSITION_CALCULATOR_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class PeoplePositionCalculator
{
public:
    PeoplePositionCalculator();

    /* Callback function for the laser scan data */
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);  
    
private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    
    std::pair<float, float> polar2cartesian_coords(std::pair<float, float> polar_coords);
    
    std::pair<float, float> middle_point(std::pair<float, float> point1, std::pair<float, float> point2); // cartesian_coords  
    
    float euclidean_distance(std::pair<float, float> point1, std::pair<float, float> point2);	// cartesian_coords  
};

#endif // PEOPLE_POSITION_CALCULATOR_H
