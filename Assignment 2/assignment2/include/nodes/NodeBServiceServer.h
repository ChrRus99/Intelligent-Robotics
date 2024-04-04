#ifndef NODE_B_PUBLISHER_H
#define NODE_B_PUBLISHER_H

#include "operations/ObjectsDetector.h"
#include <ros/ros.h>
#include <assignment2/detected_objects.h>
#include <assignment2/PickUpObject.h>
#include <assignment2/PickUpObstacle.h>

class NodeBServiceServer {
public:
    NodeBServiceServer();

    bool handle_detected_objects_service(assignment2::detected_objects::Request& req,
                                         assignment2::detected_objects::Response& res);

private:
    ros::NodeHandle nh;
    ros::ServiceServer service;
    //ObjectsDetector detector;
};

#endif // NODE_B_PUBLISHER_H
