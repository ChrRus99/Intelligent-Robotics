#include "operations/ObstaclesDetector.h"
#include "util/Point.h"
#include "util/Circle.h"
#include <stdexcept>

using namespace std;
using namespace Point;
using namespace Geometry;

ObstaclesDetector::ObstaclesDetector() : nh() {}

vector<CartesianPoint> ObstaclesDetector::detect_obstacle_positions() {
	ros::NodeHandle nh;
	sensor_msgs::LaserScanConstPtr scan_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh);

	/* HYPERPARAMETERS */
	const float FACTOR_DIST_THRESHOLD = 13.0;
	const int MIN_NUM_POINTS = 3;
	const float MAX_RADIUS = 0.5;
	const float TOLLERANCE_ERR = 0.05;


   	/* AT EACH LASER SCAN */ 
    // Extract laser scan parameters
    float angle_min = scan_msg->angle_min;
    float angle_max = scan_msg->angle_max;
    float angle_increment = scan_msg->angle_increment;
    float range_min = scan_msg->range_min;
    float range_max = scan_msg->range_max;
        
    // Extract the laser scan data
    vector<float> ranges = scan_msg->ranges;		
		
	
	/* DETECT OBSTACLES POINTS */
	vector<CartesianPoint> obstacles_points;
	
	// Mask the blind angles of the robot ([0,19] & [646,665]) -> valid range [20,645]
	int start_sight_angle = 20;
	int end_sight_angle = 645;

    // Process the current laser scan data (vector)
    for (size_t i = start_sight_angle; i <= end_sight_angle; i++) {
    	float angle = angle_min + i * angle_increment;
        float range = ranges[i];
        
		// If at this angle the scanner detects any obstacle
		if(!isinf(range)) {
			// Convert the polar_coords in cartesian_coords
			PolarPoint obstacle_polar_point = {range, angle};
			CartesianPoint obstacle_cartesian_point = to_cartesian_coordinates(obstacle_polar_point);

         	// Store the range value at this angle for the current obstacle detected
           	obstacles_points.push_back(obstacle_cartesian_point);
        }		
	}


	/* GROUP OBSTACLES POINTS IN OBSTACLES */
	// Note: no circular vector problem in this case, since angles are in range [-1.8, 1.8] rad
	vector<vector<CartesianPoint>> obstacles;
	vector<CartesianPoint> curr_obstacle;

	for(size_t i = 0; i < obstacles_points.size(); i++) {
		if(i==0) {
			// If the point x_i is the first point in the vector, then put it in the curr_obstacle vector
			curr_obstacle.push_back(obstacles_points[i]);
		} else if(i==obstacles_points.size()-1) {
			// If the point x_i is the last point in the vector, then put it in the curr_obstacle vector and store the detected obstacle
			curr_obstacle.push_back(obstacles_points[i]);
			obstacles.push_back(curr_obstacle);
			curr_obstacle.clear();
		} else {
			if(euclidean_distance(obstacles_points[i], obstacles_points[i-1]) <=
				(FACTOR_DIST_THRESHOLD * euclidean_distance(obstacles_points[i], obstacles_points[i+1]))) {
				// If d(x_i, x_i-1) <= K * d(x_i, x_i+1), then group the point x_i in the set of points of (x_i-1)
				curr_obstacle.push_back(obstacles_points[i]);
			} else {
				// Else store the last obstacle detected (if one was detected), and put the point x_i in a new curr_obstacle vector  
				if(!curr_obstacle.empty()) {
					obstacles.push_back(curr_obstacle);
					curr_obstacle.clear();
				}

				curr_obstacle.push_back(obstacles_points[i]);
			}
		}
	}


	/* FILTER OBSTACLES TO EXTRACT (THE CENTRAL POINT OF) MOVABLE CYLINDRICAL OBSTACLES ONLY */
	vector<CartesianPoint> movable_cylindrical_obstacles;

	// For each detected obstacle
	for(const auto& obstacle : obstacles) {
		// Filter obstacles that have less than a certain minimum number of points
		if(obstacle.size() < MIN_NUM_POINTS)
			continue;

		// Try to fit the distribution with a circle model
		Circle circle(CartesianPoint(0,0), 0);

		try {
			circle = fit_circle(obstacle);
		} catch (const std::invalid_argument& e) {
			// The point distribution cannot be fit with a circle, it is not a cylinder, so discard it
			continue;
		}

		// Compute the radius and the center point of the circle computed
		float radius = circle.radius;
		CartesianPoint center = circle.center;

		// Filter obstacles that are too big
		if(radius > MAX_RADIUS)
			continue;

		// Filter all obstacles whose points does not fall into the circle model fit before (with a certain tollerance error)
		float lower_bound_radius = radius - TOLLERANCE_ERR*radius;
		float upper_bound_radius = radius + TOLLERANCE_ERR*radius;
		bool is_circle = true;

		for(const auto& point : obstacle) {
			float dist_point_from_center = euclidean_distance(point, center);

			if((dist_point_from_center < lower_bound_radius) || (dist_point_from_center > upper_bound_radius)) {
				is_circle = false;
				break;
			}
		}

		// If this object is a movable cylindrical obstacle, then store its central point in the obstacle_position member variable
		if(is_circle) {
			movable_cylindrical_obstacles.push_back(center);
		}
	}

	return movable_cylindrical_obstacles;
}
