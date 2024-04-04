#include "ObstaclesDetector.h"
#include "util/Point.h"
#include "util/Circle.h"
#include <stdexcept>

using namespace std;
using namespace Point;
using namespace Geometry;

ObstaclesDetector::ObstaclesDetector() : nh()
{
	//const string TOPIC_NAME = "/scan";
	//const int MSG_QUEUE_SIZE = 100;

    // Subscribe to the laser scan topic
    //scan_sub = nh.subscribe(TOPIC_NAME, MSG_QUEUE_SIZE, &ObstaclesDetector::scan_callback, this);
}

vector<CartesianPoint> ObstaclesDetector::detect_obstacle_positions() {
	ros::NodeHandle nh;
	sensor_msgs::LaserScanConstPtr scan_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh);

	// HYPERPARAMETERS
	const double FACTOR_DIST_THRESHOLD = 15.0;
	const int MIN_NUM_POINTS = 5;
	const double MAX_RADIUS = 0.5;
	const double TOLLERANCE_ERR = 0.05;


   	/* AT EACH LASER SCAN */ 
	//ROS_INFO("NEW SCAN");

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
	
	// Mask the blind angles of the robot [20,645] (blind angles: [0,19] & [646,665])
	int start_sight_angle = 20;
	int end_sight_angle = 645;

    // Process the current laser scan data (vector)
    for (size_t i = start_sight_angle; i <= end_sight_angle; i++) {
    	float angle = angle_min + i * angle_increment;
        float range = ranges[i];
			
		//ROS_INFO("Pos: %d, Angle: %f [rad], Range: %f [m]", i, angle, range);
        
		// If at this angle the scanner detects any obstacle
		if(!isinf(range)) {
			//ROS_INFO("DETECTED OBSTACLE POINT - Pos: %d, Angle: %f [rad], Range: %f [m]", i, angle, range);

			// Convert the polar_coords in cartesian_coords
			PolarPoint obstacle_polar_point = {range, angle};
			CartesianPoint obstacle_cartesian_point = polar2cartesian_coords(obstacle_polar_point);

         	// Store the range value at this angle for the current obstacle detected
           	obstacles_points.push_back(obstacle_cartesian_point);
        }		
	}


	/* GROUP OBSTACLES POINTS IN OBSTACLES */
	// Note: no circular vector problem in this case since angles are in [-1.8, 1.8] rad
	vector<vector<CartesianPoint>> obstacles;
	vector<CartesianPoint> curr_obstacle;

	for(size_t i = 0; i < obstacles_points.size(); i++) {
		if(i==0) {
			//ROS_INFO("---> ELEM: [i=%d], POINT: [x=%0.2f, y=%0.2f], IN VECTOR: %ld", i, obstacles_points[i].x, obstacles_points[i].y, obstacles.size());
			//ROS_INFO("");
			// If the point x_i is the first point in the vector, then put it in the curr_obstacle vector
			curr_obstacle.push_back(obstacles_points[i]);
		} else if(i==obstacles_points.size()-1) {
			//ROS_INFO("---> ELEM: [i=%d], POINT: [x=%0.2f, y=%0.2f], IN VECTOR: %ld", i, obstacles_points[i].x, obstacles_points[i].y, obstacles.size());
			//ROS_INFO("");
			// If the point x_i is the last point in the vector, then put it in the curr_obstacle vector and store the detected obstacle
			curr_obstacle.push_back(obstacles_points[i]);
			obstacles.push_back(curr_obstacle);
			curr_obstacle.clear();
		} else {
			//ROS_INFO("---> DIST(x_i, x_i-1)=%0.4f, DIST(x_i,x_i+1)=%0.4f", euclidean_distance(obstacles_points[i], obstacles_points[i-1]), euclidean_distance(obstacles_points[i], obstacles_points[i+1]));
			if(euclidean_distance(obstacles_points[i], obstacles_points[i-1]) <=
				(FACTOR_DIST_THRESHOLD * euclidean_distance(obstacles_points[i], obstacles_points[i+1]))) {
				//ROS_INFO("---> ELEM: [i=%d], POINT: [x=%0.2f, y=%0.2f], IN VECTOR: %ld", i, obstacles_points[i].x, obstacles_points[i].y, obstacles.size());
				//ROS_INFO("");
				// If d(x_i, x_i-1) <= d(x_i, x_i+1), then group the point x_i in the set of points of (x_i-1)
				curr_obstacle.push_back(obstacles_points[i]);
			} else {
				// Else store the last obstacle detected (if one was detected), and put the point x_i in a new curr_obstacle vector  
				if(!curr_obstacle.empty()) {
					obstacles.push_back(curr_obstacle);
					curr_obstacle.clear();
				}

				//ROS_INFO("---> ELEM: [i=%d], POINT: [x=%0.2f, y=%0.2f], IN VECTOR: %ld", i, obstacles_points[i].x, obstacles_points[i].y, obstacles.size());
				//ROS_INFO("");
				curr_obstacle.push_back(obstacles_points[i]);
			}
		}
	}

	//ROS_INFO("NUMBER OF POTENTIAL OBSTACLES: %ld", obstacles.size());


	/* FILTER OBSTACLES TO EXTRACT THE CENTRAL POINT OF MOVABLE CYLINDRICAL OBSTACLES ONLY */
	vector<CartesianPoint> movable_cylindrical_obstacles;

	// For each detected obstacle
	for(const auto& obstacle : obstacles) {
		//ROS_INFO("NUMBER OF POINTS IN CURR OBSTACLE: %ld", obstacle.size());

		// Filter obstacles that have less than a certain minimum number of points
		if(obstacle.size() < MIN_NUM_POINTS)
			continue;

		// Try to fit the distribution with a circle model
		Circle circle(CartesianPoint(0,0), 0);

		try {
			circle = fit_circle(obstacle);
		} catch (const std::invalid_argument& e) {
			// The point distribution cannot be fitted with a circle, discard it
			continue;
		}

		// Compute the radius and the center point of the circle computed
		double radius = circle.radius;
		CartesianPoint center = circle.center;

		//ROS_INFO("RADIUS: %0.4f, CENTER: [x=%0.4f, y=%0.4f]", radius, center.x, center.y);

		// Filter obstacles that are too big
		if(radius > MAX_RADIUS)
			continue;

		// Filter all obstacles whose distribution of points does not fit a circumference model (with a certain tollerance error)
		double lower_bound_radius = radius - TOLLERANCE_ERR*radius;
		double upper_bound_radius = radius + TOLLERANCE_ERR*radius;
		bool is_circle = true;

		for(const auto& point : obstacle) {
			double dist_point_from_center = euclidean_distance(point, center);
			//ROS_INFO("---> POINT: [x=%0.2f, y=%0.2f]", point.x, point.y);
			//ROS_INFO("---> DISTANCE: %0.4f | BOUND: [%0.4f, %0.4f]", dist_point_from_center, lower_bound_radius, upper_bound_radius);
			//ROS_INFO("");

			if((dist_point_from_center < lower_bound_radius) || (dist_point_from_center > upper_bound_radius)) {
				is_circle = false;
				break;
			}
		}

		// If this object is a movable cylindrical obstacle, then store its central point in the obstacle_position member variable
		if(is_circle) {
			movable_cylindrical_obstacles.push_back(center);

			//ROS_INFO("DETECTED MOVABLE CYLINDRICAL OBSTACLE NUMBER [%ld] IN: [x: %f, y: %f]", movable_cylindrical_obstacles.size(), center.x, center.y);
		}
	}

	return movable_cylindrical_obstacles;
}