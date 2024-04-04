#include "PeoplePositionCalculator.h"

using namespace std;

PeoplePositionCalculator::PeoplePositionCalculator() : nh()
{
	const string TOPIC_NAME = "/scan";
	const int MSG_QUEUE_SIZE = 100;
    
    // Subscribe to the laser scan topic
    scan_sub = nh.subscribe(TOPIC_NAME, MSG_QUEUE_SIZE, &PeoplePositionCalculator::scan_callback, this);
}

void PeoplePositionCalculator::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{    	
   	/* AT EACH LASER SCAN */
	ROS_INFO("NEW SCAN");
   		
    // Extract laser scan parameters
    float angle_min = scan_msg->angle_min;
    float angle_max = scan_msg->angle_max;
    float angle_increment = scan_msg->angle_increment;
    float range_min = scan_msg->range_min;
    float range_max = scan_msg->range_max;
        
    // Extract the laser scan data
    vector<float> ranges = scan_msg->ranges;        
		
	// Compute the laser scan vector size
	int data_vector_size = ((angle_max - angle_min) / angle_increment) + 1;
	// equivalent: int data_vector_size = ranges.size()
	ROS_INFO("Laser scan vector size: %d", data_vector_size);
		
		
	/* DETECT PEOPLE'S LEGS POLAR_COORDS POINTS */
	// E.g. leg_1: polar_coords_vect=[(i_range, i_angle), ..., (j_range, j_angle)]>
	vector<vector<pair<float, float>>> legs_data;
	vector<pair<float, float>> curr_leg_data;
		
    // Process the current laser scan data (vector) to compute the position of each leg
    for (size_t i = 0; i < ranges.size(); ++i) {
    	float angle = angle_min + i * angle_increment;
        float range = ranges[i];
			
        //ROS_INFO("Angle: %f [rad], Range: %f [m]", angle, range);
            
        // If at this angle the scanner does not detect any leg
        if(isinf(range)) {
         	// Store the last leg detected (if one was detected)
           	if(!curr_leg_data.empty()) {
		       	legs_data.push_back(curr_leg_data);
		       	curr_leg_data.clear();
           	}
        } else {
           	// Store the range value at this angle for the current leg detected
           	curr_leg_data.push_back({range, angle});
            	
           	//ROS_INFO("DETECTED LEG - Angle: %f [rad], Range: %f [m]", angle, range);
        }
	}
        
    // (If not done yet) store the last leg detected
    if(!curr_leg_data.empty()) {
    	legs_data.push_back(curr_leg_data);
    }
               
        
    /* FOR EACH LEG DETECTED, COMPUTE THE CENTRAL CARTESIAN_COORDS POINT OF THE LEG */
    vector<pair<float, float>> legs_central_points;	// cartesian_coords
        
    // For each leg detected       
    for(const auto& leg : legs_data) {
    	// Compute the central polar_coords point
      	pair<float, float> central_point_polar_coords = (leg.size() % 2 == 0) ? leg[leg.size() / 2] : leg[(leg.size()-1) / 2];
        	
    	// Convert the central polar coords in cartesian_coords, and store it
    	pair<float, float> leg_position = polar2cartesian_coords(central_point_polar_coords);
     	legs_central_points.push_back(leg_position);
        	
      	// ROS_INFO("DETECTED LEG - Leg central point: [x=%f, y=%f]", leg_position.first, leg_position.second);
	}
        
    /* FOR EACH PERSON, COMPUTE THE MIDDLE CARTESIAN_COORDS POINT BETWEEN ITS COUPLE OF LEGS */
    /* Assumptions: 
   		- Since the N=3 people are facing the robot, and since to compute a person position we must detect its 2 legs,
          we assume to detect an even number of legs
        - Moreover, we assume people's legs not crossing each others
     */
     
    // Check whether a valid number of legs was detected
    if(legs_central_points.size() % 2 != 0) {
    	ROS_INFO("ERROS: Uneven number of legs detected");
    	return;
    }
        
    vector<pair<float, float>> people_positions;
    int first_idx = 0;
    int last_idx = legs_central_points.size() - 1;
    int counter_people = 0;
    
    // Given the 1st leg, if the nearest leg is the last one (instead of the 2nd one) in the legs_central_points vector, 
    // then adapt indexes (circular vector)
    if(euclidean_distance(legs_central_points[0], legs_central_points[1]) >
    	euclidean_distance(legs_central_points[0], legs_central_points[legs_central_points.size()-1])) {
    	first_idx = 1;
     	last_idx = legs_central_points.size();
    }
    
    // Couple a pair of legs for each person
    for(int i=first_idx; i<last_idx; i+=2) {    
		pair<float, float> person_position;
			
    	// Compute the position of a person as the middle point between its 2 legs
    	if(i >= legs_central_points.size()-1) {
    		// Couple 1st leg and last leg 
		   	person_position = middle_point(
		 		legs_central_points.at(i), 
		 		legs_central_points.at(0)
		    );
		} else {
			// Couple current leg with next leg
			person_position = middle_point(
				legs_central_points.at(i), 
				legs_central_points.at(i+1)
			);
		}
		
		// Store the position of the detected person
		people_positions.push_back(person_position);
		counter_people++;
        	
        ROS_INFO("DETECTED PERSON [%d]: Person middle point: [x=%f, y=%f]", counter_people, person_position.first, person_position.second);
    }
}

pair<float, float> PeoplePositionCalculator::polar2cartesian_coords(pair<float, float> polar_coords) 
{
   	float radius = polar_coords.first;
   	float angle = polar_coords.second;
    
   	float x = radius * cos(angle);
   	float y = radius * sin(angle);
   	return pair<float, float>{x, y};
}
    
pair<float, float> PeoplePositionCalculator::middle_point(pair<float, float> point1, pair<float, float> point2) // cartesian_coords
{
  	float x_middle = (point1.first + point2.first) / 2.0f;
   	float y_middle = (point1.second + point2.second) / 2.0f;
    	
   	return pair<float, float>{x_middle, y_middle};
}

float PeoplePositionCalculator::euclidean_distance(pair<float, float> point1, pair<float, float> point2)	// cartesian_coords
{
	float dx = point2.first - point1.first;
	float dy = point2.second - point1.second;
	
	return sqrt(dx * dx + dy * dy);
}
