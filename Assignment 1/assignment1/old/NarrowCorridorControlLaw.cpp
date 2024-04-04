#include "NarrowCorridorControlLaw.h"

using namespace std;

NarrowCorridorControlLaw::NarrowCorridorControlLaw() : nh() 
{
    ROS_INFO("START");
    // Set up subscriber for laser scan data
    //laser_sub = nh.subscribe("/scan", 1, &NarrowCorridorControlLaw::laserCallback, this);

    // Set up publisher for velocity commands
    wheels_velocity_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1);

    // Initialize wheel speeds
    left_wheel_speed = INITIAL_SPEED;  // Initial left wheel speed
    right_wheel_speed = INITIAL_SPEED; // Initial right wheel speed

    control_law_navigation();   // DA CHIAMARE NEL MAIN COME controller.control_law_navigation()
}

// Helper function to convert an angle to the corresponding index in the laser data array
int angle_to_index(float angle, float angle_min, float angle_increment)
{
    // Calculate the index corresponding to the specified angle
    int index = static_cast<int>((angle - angle_min) / angle_increment);
    return index;
}

float index_to_angle(int index, const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{
    // Ensure the index is within valid range
    index = max(0, min(index, static_cast<int>(laser_msg->ranges.size()) - 1));

    // Extract angle_min and angle_increment from laser scan data
    float angle_min = laser_msg->angle_min;
    float angle_increment = laser_msg->angle_increment;

    // Calculate the angle corresponding to the specified index
    float angle = angle_min + index * angle_increment;

    return angle;
}


// Helper function to calculate the average distance at a given index in the laser data array
float calculate_average_distance(int index, const vector<float>& laser_ranges)
{
    // Ensure the index is within valid range
    index = max(0, min(index, static_cast<int>(laser_ranges.size()) - 1));

    // Consider a small window around the specified index (e.g., 5 data points)
    int window_size = 10;
    int start_index = max(0, index - window_size / 2);
    int end_index = min(static_cast<int>(laser_ranges.size()) - 1, start_index + window_size);

    //ROS_INFO("Start index: %d, End index: %d", start_index, end_index);

    // Calculate the average distance in the specified window
    float sum_distance = 0.0;

    for(int i = start_index; i <= end_index; ++i) {
        sum_distance += laser_ranges[i];
    }
    
    float average_distance = sum_distance / (end_index - start_index + 1);

    return average_distance;
}

pair<float, float> measure_wall_distances(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{
    const float BLIND_ANGLE = 0.0; // 20.0  // DA TOGLIERE FA CASINO

    // Extract angle_min, angle_max, and angle_increment from laser scan data
    float angle_min = laser_msg->angle_min + BLIND_ANGLE;
    float angle_max = laser_msg->angle_max - BLIND_ANGLE;
    float angle_increment = laser_msg->angle_increment;

    // Specify the angles (in radians) on both sides of the robot (orthogonal directions)
    float left_angle = M_PI / 2.0;  // 90 degrees (left side)
    float right_angle = -M_PI / 2.0; // -90 degrees (right side)

    // Calculate the index corresponding to the specified angles
    int left_index = angle_to_index(left_angle, angle_min, angle_increment);
    int right_index = angle_to_index(right_angle, angle_min, angle_increment);
    
    //ROS_INFO("Left index: %d, Right index: %d", left_index, right_index);

    // Convert indices to angles
    //float left_angle_rad = index_to_angle(left_index, laser_msg);
    //float right_angle_rad = index_to_angle(right_index, laser_msg);
    //ROS_INFO("Start angle (given): %f, End angle (given): %f", left_angle, right_angle);
    //ROS_INFO("Start angle (computed): %f, End angle (computed): %f", left_angle_rad, right_angle_rad);

    // Get laser data from the last received scan message
    // Remove the first and last 20 measurements (blind angles)
    vector<float> laser_ranges(laser_msg->ranges.begin() + BLIND_ANGLE, laser_msg->ranges.end() - BLIND_ANGLE);

    // Calculate the average distances at the specified angles
    float left_distance = calculate_average_distance(left_index, laser_ranges);
    float right_distance = calculate_average_distance(right_index, laser_ranges);

    ROS_INFO("Left dist: %f [m], Right dist: %f [m]", left_distance, right_distance);

    return pair<float, float>(left_distance, right_distance);
}

void NarrowCorridorControlLaw::adjust_wheels_velocity(const pair<float, float>& wall_distances)
{
    const float ADJUSTING_FACTOR = 1/500.0;
    const float SMOOTHING_FACTOR = 1/100.0;

    float left_distance = wall_distances.first;
    float right_distance = wall_distances.second;

    float adjusting_step = INITIAL_SPEED * ADJUSTING_FACTOR;
    float smoothing_step = INITIAL_SPEED * SMOOTHING_FACTOR;

    if(fabs(left_wheel_speed - right_wheel_speed) < 0.005) {
        // Adjust wheel speeds based on wall proximity
        if((left_distance - right_distance) > 0) {
            // Robot is getting close to the right wall, increase right wheel speed, decrease left wheel speed
            left_wheel_speed -= adjusting_step;
            right_wheel_speed += adjusting_step;
            ROS_INFO("Adjusted deviation to the right");
        } else if((left_distance - right_distance) < 0) {
            // Robot is getting close to the left wall, increase left wheel speed, decrease right wheel speed
            left_wheel_speed += adjusting_step;
            right_wheel_speed -= adjusting_step;
            ROS_INFO("Adjusted deviation to the left");
        } else {
            // Robot is in the center of the corridor, maintain current speeds
        }
    } else {    // Smooth the turn
        if(left_wheel_speed > right_wheel_speed) {
            left_wheel_speed -= smoothing_step;
            right_wheel_speed += smoothing_step;
        } else {
            left_wheel_speed += smoothing_step;
            right_wheel_speed -= smoothing_step;
        }
    }

    // Ensure wheel speeds are within limits (adjust as needed)
    //left_wheel_speed = max(0.0, min(1.0, left_wheel_speed));
    //right_wheel_speed = max(0.0, min(1.0, right_wheel_speed));

    ROS_INFO("Wheel speed: Left: [%f], Right: [%f]", left_wheel_speed, right_wheel_speed);
}

bool check_narrow_corridor(const pair<float, float>& wall_distances, float corridor_width)
{
    float left_distance = wall_distances.first;
    float right_distance = wall_distances.second;

    return (left_distance + right_distance) < corridor_width;
}

void NarrowCorridorControlLaw::control_law_navigation()
//void NarrowCorridorControlLaw::laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg) 
{
    ROS_INFO("CONTROL LAW NAVIGATION");

    const float MAX_CORRIDOR_WIDTH = 1.2;
    const float STEERING_FACTOR = 3; // 2

    bool was_in_narrow_corridor = false;


    geometry_msgs::Twist cmd_vel;
    
    while(1) {
        ROS_INFO("NEW ITERATION");

        sensor_msgs::LaserScanConstPtr laser_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh);

        // Find the distance from the wall on the left and right sides 
        pair<float, float> wall_distances = measure_wall_distances(laser_msg);
        
        // Check whether the robot is in the narrow corridor
        bool is_in_narrow_corridor = check_narrow_corridor(wall_distances, MAX_CORRIDOR_WIDTH);
        ROS_INFO("Is in narrow corridor: %d", is_in_narrow_corridor);

        // Control law to pass the corridor
        if(is_in_narrow_corridor) {
            // If enter in narrow corridor
            was_in_narrow_corridor = true;

            // Control law: adjust wheel speeds based on laser data
            adjust_wheels_velocity(wall_distances);

            cmd_vel.linear.x = (left_wheel_speed + right_wheel_speed) / 2.0;  // Average linear velocity
            cmd_vel.angular.z = STEERING_FACTOR * ((right_wheel_speed - left_wheel_speed) / 0.3);   // Average angular velocity (steering speed)
            wheels_velocity_pub.publish(cmd_vel);
        } else if(was_in_narrow_corridor && !is_in_narrow_corridor) {
            // If exited from the narrow corridor
            ROS_INFO("Is exited from narrow corridor");

            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            wheels_velocity_pub.publish(cmd_vel);
            break;
        } else {
            // If not entered in the narrow corridor yet
            ROS_INFO("Is not entered in narrow corridor yet");

            cmd_vel.linear.x = 0.5;
            cmd_vel.angular.z = 0.0;
            wheels_velocity_pub.publish(cmd_vel);
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "narrow_passage_controller");

    NarrowCorridorControlLaw controller;
    
    return 0;
}