#include "operations/DeliveryTablesDetector.h"
#include "util/Point.h"
#include "util/ComputerVisionUtils.h"
#include "operations/ObstaclesDetector.h"
#include "operations/TiagoHeadController.h"

using namespace std;
using namespace cv;
using namespace Point;
using namespace ObjectsModels;


/* PUBLIC PROGRAM FUNCTIONS */
DeliveryTablesDetector::DeliveryTablesDetector(const Pose& robot_pose_in_map)
    : robot_pose_in_map(robot_pose_in_map)
{
    detect_delivery_tables_positions();
    detect_delivery_tables_colors();
}

DeliveryTable DeliveryTablesDetector::get_target_delivery_table(const PickUpObject& target_object) 
{
    ROS_INFO("(DELIVERY TABLES DETECTOR): Target object: %s", to_string(target_object));

    for(const auto& delivery_table : detected_delivery_tables) {
        if(delivery_table.get_color() == target_object.get_color()) {
            ROS_INFO("(DELIVERY TABLES DETECTOR): Corresponding target delivery table: %s", to_string(delivery_table));
            return delivery_table;
        }
    }

    return DeliveryTable(DeliveryTable::TableType::NONE);
}


/* PRIVATE PROGRAM FUNCTIONS */
void DeliveryTablesDetector::detect_delivery_tables_positions()
{
    // Detect delivery tables using laser scan (in robot reference frame: base: {(0, -1), (1, 0)})
    ObstaclesDetector obstacles_detector; 
    vector<CartesianPoint> detected_obstacles = obstacles_detector.detect_obstacle_positions();

    // Convert the delivery table position from robot reference frame to map reference frame: base: {(-1, 0), (0, -1)})
    Position robot_position_in_map = to_position(robot_pose_in_map);

    const double DELIVERY_OFFSET_RHO = 0.49;
    const double DELIVERY_FACTOR_THETA = 2.5;

    ROS_INFO("(DELIVERY TABLES DETECTOR): Robot has detected [%ld] delivery tables:", detected_obstacles.size());
    for (const auto& obstacle : detected_obstacles) {
        // Delivery table position in robot reference frame
        PolarPoint translation = to_polar_coordinates(CartesianPoint(obstacle.x, obstacle.y));
        ROS_INFO("(DELIVERY TABLES DETECTOR): Delivery table in position (robot reference frame): Cartesian coordinates: %s, Polar coordinates: %s",
            to_string(obstacle), to_string(translation));

        // Convert the delivery table position from robot reference frame to map reference frame
        Pose delivery_table(
            robot_position_in_map.x + (translation.rho - DELIVERY_OFFSET_RHO) * sin(translation.theta),
            robot_position_in_map.y + (translation.rho - DELIVERY_OFFSET_RHO) * cos(translation.theta),
            -((translation.theta / DELIVERY_FACTOR_THETA) * (180.0 / M_PI)) + 90.0
        );
        ROS_INFO("(DELIVERY TABLES DETECTOR): Delivery table in position (map reference frame): %s", to_string(delivery_table));

        // Store the detected delivery table position (in the map reference frame)
        detected_delivery_tables.push_back(DeliveryTable(delivery_table));
    }
}

void DeliveryTablesDetector::detect_delivery_tables_colors() 
{
    // Move the robot's head in position for allowing the robot's camera to detect the obstacles
    TiagoHead tiago_head;
  	tiago_head.move_head(-0.45);

    // Get the image from robot's camera
    Mat image = cv_bridge::toCvCopy(
        ros::topic::waitForMessage<sensor_msgs::Image>("/xtion/rgb/image_raw", nh), 
        sensor_msgs::image_encodings::BGR8
    )->image;

    // Resize the image
    resize(image, image, Size(), 0.8, 0.8);

    // Convert image to HSV
    Mat image_HSV;
    Mat image_centroids;
    Mat mask_blue_points;
    Mat mask_green_points;
    Mat mask_red_points;

    // Convert image to HSV
    cvtColor(image, image_HSV, COLOR_BGR2HSV);

    // Compute thresholded images
    inRange(image_HSV, Scalar(110, 50, 50), Scalar(130, 255, 255), mask_blue_points);
    inRange(image_HSV, Scalar(36, 25, 25), Scalar(70, 255, 255), mask_green_points);
    inRange(image_HSV, Scalar(0, 50, 50), Scalar(10, 255, 255), mask_red_points);

    // Get the points of each thresholded image
    vector<Point2i> blue_points = get_foreground_pixels(mask_blue_points);
    vector<Point2i> green_points = get_foreground_pixels(mask_green_points);
    vector<Point2i> red_points = get_foreground_pixels(mask_red_points);

    // Compute the centroids of each  the points of each thresholded image
    Point2i blue_centroid = centroid_pixelset(blue_points);
    Point2i green_centroid = centroid_pixelset(green_points);
    Point2i red_centroid = centroid_pixelset(red_points);

    // Draw each centroid in the image of centroids
    image_centroids = image.clone();
    circle(image_centroids, blue_centroid, 5, Scalar(255, 100, 100), -1);
    circle(image_centroids, green_centroid, 5, Scalar(100, 255, 100), -1);
    circle(image_centroids, red_centroid, 5, Scalar(100, 100, 255), -1);

    // Sort in ascending order the centroids depending on their position along the x axis of the image
    // Note: in the image the position (0, 0) is located at the top-left corner of the image. 
    // The coordinates (x, y) follow the Cartesian coordinate system, where the x-axis increases from
    // left to right and the y-axis increases from top to bottom.
    vector<pair<Color, Point2i>> color_centroids;
    color_centroids.push_back({Color::BLUE, blue_centroid});
    color_centroids.push_back({Color::GREEN, green_centroid});
    color_centroids.push_back({Color::RED, red_centroid});

    sort(color_centroids.begin(), color_centroids.end(), [](const auto& a, const auto& b) {
        return a.second.x < b.second.x;
    });

    // Sort in acending order the poses of each detected delivery table depending on their position along
    // the x axis of the map
    sort(detected_delivery_tables.begin(), detected_delivery_tables.end(), [](const auto& a, const auto& b) {
        return a.get_pose().x < b.get_pose().x;
    });

    // Associate each color to the corresponding delivery table (by corresponding index)
    ROS_INFO("(DELIVERY TABLES DETECTOR): Association color to delivery table:");
    for(size_t i = 0; i < detected_delivery_tables.size(); i++) {
        detected_delivery_tables[i].set_color(color_centroids[i].first);
        ROS_INFO("(DELIVERY TABLES DETECTOR): Colored delivery table: %s", to_string(detected_delivery_tables[i]));
    }

    // Show the source and the processed images 
    vector<Mat> images;
	vector<string> labels;
    int num_rows = 2;
	int num_cols = 3;

	labels.push_back("Source Image BGR");
	images.push_back(image);
	labels.push_back("Converted Image HSV");
	images.push_back(image_HSV);
    labels.push_back("Centroids Image");
	images.push_back(image_centroids);
    labels.push_back("Mask Of Blue Points");
	images.push_back(mask_blue_points);
    labels.push_back("Mask Of Green Points");
	images.push_back(mask_green_points);
    labels.push_back("Mask Of Red Points");
	images.push_back(mask_red_points);

    ROS_INFO("(DELIVERY TABLES DETECTOR): Camera image show: press a key to continue");
	subplot("Camera Feed", images, labels, num_rows, num_cols, true);
}


/* HELPER FUNCTIONS */
vector<Point2i> get_foreground_pixels(const Mat& mask) 
{
    vector<Point2i> foreground_points;

    // Extract all foreground pixels in the mask
    for (size_t y = 0; y < mask.rows; y++) {
        for (size_t x = 0; x < mask.cols; x++) {
            if (mask.at<uchar>(y, x) > 0) {
                foreground_points.push_back(Point2i(x, y));
            }
        }
    }
    return foreground_points;
}

Point2i centroid_pixelset(const vector<Point2i>& points) {
    Point2i centroid = {0, 0};

    // Sum up the coordinates of all points in the pointset
    for (const auto& point : points) {
        centroid.x += point.x;
        centroid.y += point.y;
    }

    // Divide by the number of points to get the average coordinates
    size_t num_points = points.size();
    if (num_points > 0) {
        centroid.x /= num_points;
        centroid.y /= num_points;
    }

    return centroid;
}