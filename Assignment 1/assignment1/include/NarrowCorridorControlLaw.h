#ifndef NARROW_PASSAGE_CONTROLLER_H
#define NARROW_PASSAGE_CONTROLLER_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class NarrowCorridorControlLaw {
public:
    NarrowCorridorControlLaw();

    /* PUBLIC PROGRAM FUNCTIONS */
    void control_law_navigation();

private:
    ros::NodeHandle nh;
    ros::Publisher wheels_velocity_pub;
    float left_wheel_speed;
    float right_wheel_speed;
    const float INITIAL_SPEED = 0.35; 

    /* PRIVATE PROGRAM FUNCTIONS */
    std::pair<float, float> measure_wall_distances(const sensor_msgs::LaserScan::ConstPtr& laser_msg);
    void adjust_wheels_velocity(const std::pair<float, float>& wall_distances);
};

/* HELPER FUNCTIONS */
/* Helper function to convert an angle to the corresponding index in the laser data array */
int angle_to_index(float angle, float angle_min, float angle_increment);

/* Helper function to calculate the average distance in a window around a given index in the laser data array */
float calculate_average_distance(int index, const std::vector<float>& laser_ranges);

/* Helper function to check whether the robot is in the narrow corridor */
bool check_narrow_corridor(const std::pair<float, float>& wall_distances, float corridor_width);

#endif // NARROW_PASSAGE_CONTROLLER_H
