#ifndef DELIVERY_TABLE_DETECTOR_H
#define DELIVERY_TABLE_DETECTOR_H

#include "models/ObjectsModels.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

class DeliveryTablesDetector
{
public:
    DeliveryTablesDetector(const Point::Pose& robot_pose_in_map);        

    /* PUBLIC PROGRAM FUNCTIONS */
	/* This function returns the position of the corresponding delivery table for the target object */
    ObjectsModels::DeliveryTable get_target_delivery_table(const ObjectsModels::PickUpObject& target_object);    
    
private:
    ros::NodeHandle nh;
    Point::Pose robot_pose_in_map;
    std::vector<ObjectsModels::DeliveryTable> detected_delivery_tables;

    /* PRIVATE PROGRAM FUNCTIONS */
    /* This function detect the delivery tables using the robot's laser scan */
    void detect_delivery_tables_positions();
    /* This function detects the color of each detected delivery table using the robot's camera */
    void detect_delivery_tables_colors();
};

/* HELPER FUNCTIONS */
/* This function gets the foreground pixels' points of the given mask */
std::vector<cv::Point2i> get_foreground_pixels(const cv::Mat& mask);
/* This function computes the centroid of a distribution of pixels (i.e. of a pointset) */
cv::Point2i centroid_pixelset(const std::vector<cv::Point2i>& points);

#endif // DELIVERY_TABLE_DETECTOR_H