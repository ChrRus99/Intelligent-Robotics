#include "ros/ros.h"
#include "PeoplePositionCalculator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "people_position_calculator");

    PeoplePositionCalculator people_position_calculator;

    ros::spin();

    return 0;
}

