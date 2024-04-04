#ifndef NODE_B_PUBLISHER_H
#define NODE_B_PUBLISHER_H

#include "operations/ObjectsDetector.h"
#include <ros/ros.h>
#include <custom_msgs/detected_objects.h>
#include <custom_msgs/PickUpObject.h>
#include <custom_msgs/PickUpObstacle.h>

class NodeBServiceServer {
public:
    NodeBServiceServer();

    bool handle_detected_objects_service(custom_msgs::detected_objects::Request& req,
                                         custom_msgs::detected_objects::Response& res);

private:
    ros::NodeHandle nh;
    ros::ServiceServer service;
    //ObjectsDetector detector;
};

#endif // NODE_B_PUBLISHER_H
