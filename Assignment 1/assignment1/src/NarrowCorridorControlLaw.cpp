#include "NarrowCorridorControlLaw.h"

using namespace std;

NarrowCorridorControlLaw::NarrowCorridorControlLaw() : nh() 
{
    // Set up publisher for velocity commands
    wheels_velocity_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1);

    // Initialize left and right wheel speeds
    left_wheel_speed = INITIAL_SPEED;
    right_wheel_speed = INITIAL_SPEED;
}

void NarrowCorridorControlLaw::control_law_navigation()
{
    /* HYPERPARAMETERS */
    const float MAX_CORRIDOR_WIDTH = 1.2;
    const float STEERING_FACTOR = 3;

    
    /* LOOP NARROW CORRIDOR CONTROL LAW */
    bool was_in_narrow_corridor = false;
    geometry_msgs::Twist cmd_vel;

    while(1) {
        // Read a new scan of sensor laser data
        sensor_msgs::LaserScanConstPtr laser_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh);

        // Find the distance of the robot from the left and right walls 
        pair<float, float> wall_distances = measure_wall_distances(laser_msg);
        
        // Check whether the robot is in the narrow corridor
        bool is_in_narrow_corridor = check_narrow_corridor(wall_distances, MAX_CORRIDOR_WIDTH);
        ROS_INFO("(CONTROL LAW): Robot is in narrow corridor: %d", is_in_narrow_corridor);

        // Control law to pass the corridor
        if(is_in_narrow_corridor) {
            // If robot enters the narrow corridor
            was_in_narrow_corridor = true;

            // Control law: adjust wheel speeds based on walls distances
            adjust_wheels_velocity(wall_distances);

            // Send adjusted wheel speeds to the robot
            cmd_vel.linear.x = (left_wheel_speed + right_wheel_speed) / 2.0;                        // average linear velocity
            cmd_vel.angular.z = STEERING_FACTOR * ((right_wheel_speed - left_wheel_speed) / 0.3);   // (corrected) average angular velocity (steering speed)
            wheels_velocity_pub.publish(cmd_vel);
        } else if(was_in_narrow_corridor && !is_in_narrow_corridor) {
            // If robot exited from the narrow corridor
            ROS_INFO("(CONTROL LAW): Robot is exited from narrow corridor");

            // Stop the robot
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            wheels_velocity_pub.publish(cmd_vel);
            break;
        } else {
            // If not entered in the narrow corridor yet
            ROS_INFO("(CONTROL LAW): Robot is not entered in narrow corridor yet");

            // Send constant wheel speeds to the robot (straight trajectory)
            cmd_vel.linear.x = INITIAL_SPEED;
            cmd_vel.angular.z = 0.0;
            wheels_velocity_pub.publish(cmd_vel);
        }
    }
}

pair<float, float> NarrowCorridorControlLaw::measure_wall_distances(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{
    // Note: here we neglect the blind angles, because they are not considered

    // Extract laser scan parameters
    float angle_min = laser_msg->angle_min;
    float angle_max = laser_msg->angle_max;
    float angle_increment = laser_msg->angle_increment;

    // Specify the angles (in radians) on both sides of the robot (orthogonal directions)
    float left_angle = M_PI / 2.0;  // 90 degrees (left side)
    float right_angle = -M_PI / 2.0; // -90 degrees (right side)

    // Calculate the index corresponding to the specified angles
    int left_index = angle_to_index(left_angle, angle_min, angle_increment);
    int right_index = angle_to_index(right_angle, angle_min, angle_increment);

    // Extract the laser scan data (from the last received scan message)
    vector<float> laser_ranges(laser_msg->ranges.begin(), laser_msg->ranges.end());

    // Calculate the average distances at the specified angles
    float left_distance = calculate_average_distance(left_index, laser_ranges);
    float right_distance = calculate_average_distance(right_index, laser_ranges);

    ROS_INFO("(CONTROL LAW): Left dist: %0.4f [m], Right dist: %0.4f [m]", left_distance, right_distance);

    return pair<float, float>(left_distance, right_distance);
}

void NarrowCorridorControlLaw::adjust_wheels_velocity(const pair<float, float>& wall_distances)
{
    // Set the adjusting factors
    const float ADJUSTING_FACTOR = 1/500.0;
    const float SMOOTHING_FACTOR = 1/100.0;
    const float MAX_WHEELS_SPEED_DIFFERENCE = 0.005;

    float adjusting_step = INITIAL_SPEED * ADJUSTING_FACTOR;
    float smoothing_step = INITIAL_SPEED * SMOOTHING_FACTOR;

    // Extract the wall distances
    float left_distance = wall_distances.first;
    float right_distance = wall_distances.second;

    
    // Prevent that speeds of the two wheels from diverging, causing the robot to rotate in itself
    if(fabs(left_wheel_speed - right_wheel_speed) < MAX_WHEELS_SPEED_DIFFERENCE) {
        // Adjust wheel speeds based on wall proximity
        if((left_distance - right_distance) > 0) {
            // If robot is getting close to the right wall, then increase right wheel speed, decrease left wheel speed
            left_wheel_speed -= adjusting_step;
            right_wheel_speed += adjusting_step;
        } else if((left_distance - right_distance) < 0) {
            // If robot is getting close to the left wall, then increase left wheel speed, decrease right wheel speed
            left_wheel_speed += adjusting_step;
            right_wheel_speed -= adjusting_step;
        } else {
            // Robot is in the center of the corridor, maintain current speeds
        }
    } else {
        // Smooth the steering of the robot realigning the speed of the wheels to stay on a linear trajectory
        if(left_wheel_speed > right_wheel_speed) {
            left_wheel_speed -= smoothing_step;
            right_wheel_speed += smoothing_step;
        } else {
            left_wheel_speed += smoothing_step;
            right_wheel_speed -= smoothing_step;
        }
    }

    // Ensure wheel speeds are within limits
    left_wheel_speed = max(0.0, min(1.0, static_cast<double>(left_wheel_speed)));
    right_wheel_speed = max(0.0, min(1.0, static_cast<double>(right_wheel_speed)));

    ROS_INFO("(CONTROL LAW): Wheel speed: Left: [%0.4f], Right: [%0.4f]", left_wheel_speed, right_wheel_speed);
}

bool check_narrow_corridor(const pair<float, float>& wall_distances, float corridor_width)
{
    // Check whether the distance between the two walls is lower than a given threshold
    float left_distance = wall_distances.first;
    float right_distance = wall_distances.second;

    return (left_distance + right_distance) < corridor_width;
}

int angle_to_index(float angle, float angle_min, float angle_increment)
{
    // Calculate the index corresponding to the specified angle
    int index = static_cast<int>((angle - angle_min) / angle_increment);
    return index;
}

float calculate_average_distance(int index, const vector<float>& laser_ranges)
{
    // Ensure the index is within valid range
    index = max(0, min(index, static_cast<int>(laser_ranges.size()) - 1));

    // Consider a small window around the specified index
    const int WINDOW_SIZE = 10;
    int start_index = max(0, index - WINDOW_SIZE / 2);
    int end_index = min(static_cast<int>(laser_ranges.size()) - 1, start_index + WINDOW_SIZE);

    // Calculate the average distance in the specified window
    float sum_distance = 0.0;

    for(int i = start_index; i <= end_index; i++) {
        sum_distance += laser_ranges[i];
    }
    
    float average_distance = sum_distance / (end_index - start_index + 1);

    return average_distance;
}
