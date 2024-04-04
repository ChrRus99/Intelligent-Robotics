#include "operations/ObjectsDetector.h"

using namespace std;
using namespace Point;
using namespace ObjectsModels;

ObjectsDetector::ObjectsDetector() : nh("~"), tfBuffer(), tfListener(tfBuffer)
{ 
  	tiago_head.move_head(-0.65);

    detect_objects_on_pick_up_table(); 
}

void ObjectsDetector::detect_objects_on_pick_up_table()
{
    ros::NodeHandle nh; 
    apriltag_ros::AprilTagDetectionArrayConstPtr msg = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections", nh);

    objects.clear();
    obstacles.clear();
    
    ROS_INFO("Processing object detection...");
    if (msg->detections.empty()) {
        ROS_INFO("No objects detected.");
        return;
    }
    ROS_INFO("Detected %lu objects.", msg->detections.size());
    
    for (const auto& detection : msg->detections) {
        geometry_msgs::PoseStamped object_pose_camera;  // Camera reference frame
        object_pose_camera.header = msg->header;
        geometry_msgs::PoseStamped object_pose_robot;   // Robot reference frame
        geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("base_link", object_pose_camera.header.frame_id, ros::Time(0));
        object_pose_camera.pose.position = detection.pose.pose.pose.position;
        object_pose_camera.pose.orientation = detection.pose.pose.pose.orientation;
        ROS_INFO("Detected object pose in the camera frame [%s] for object ID [%d]: [x=%f, y=%f, z=%f]",
                 object_pose_camera.header.frame_id.c_str(),
                 detection.id[0],
                 object_pose_camera.pose.position.x,
                 object_pose_camera.pose.position.y,
                 object_pose_camera.pose.position.z);
        try {
        	// Make the transofrmation
            tf2::doTransform(object_pose_camera, object_pose_robot, transformStamped);
            
            ROS_INFO("Transformed pose in the robot frame [%s] for object ID [%d]: [x=%f, y=%f, z=%f]",
                     transformStamped.header.frame_id.c_str(),
                     detection.id[0],
                     object_pose_robot.pose.position.x,
                     object_pose_robot.pose.position.y,
                     object_pose_robot.pose.position.z);
            // Create the position of the detected object
            Position position(object_pose_robot.pose.position.x,
                     object_pose_robot.pose.position.y,
                     object_pose_robot.pose.position.z);

            if (detection.id[0] == 1 || detection.id[0] == 2 || detection.id[0] == 3) {
	            // Create the object and store position and ID                         
	            PickUpObject object(detection.id[0], position);

	            //Add the object pose to the objects vecotor
	            objects.push_back(object);
			}
            else {
            	// Create the obstacle and store position and ID
            	PickUpObstacle obstacle(detection.id[0], position);      

            	//Add the object pose to the obstacles vector
            	obstacles.push_back(obstacle); 
            }        
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Failed to transform object pose from camera frame to robot frame: %s", ex.what());
            continue;
        }
    }
    ROS_INFO("Object detection terminated");    
}

