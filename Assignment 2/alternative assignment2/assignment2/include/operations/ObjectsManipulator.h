#ifndef OBJECTS_MANIPULATOR_H
#define OBJECTS_MANIPULATOR_H

#include "util/Point.h"
#include "util/PointParser.h"
#include "models/ObjectsModels.h"
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <string>
#include <vector>
#include <map>
#include "gazebo_ros_link_attacher/Attach.h"

class ObjectsManipulator
{
public:
    /* Constructor for picking-up target_object routine */
    ObjectsManipulator(ObjectsModels::PickUpObject target_object, std::vector<ObjectsModels::PickUpObject> detected_objects, std::vector<ObjectsModels::PickUpObstacle> detected_obstacles)   
        : 	target_object(target_object),
        	detected_objects(detected_objects),
        	detected_obstacles(detected_obstacles),
        	group_arm_torso("arm_torso"),
        	planning_scene_interface(),
        	collision_objects(),
        	attachClient(nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach")),
        	detachClient(nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach"))
    {}
    /* Constructor for placing target_object routine */
    ObjectsManipulator(ObjectsModels::PickUpObject target_object)
    	: 	ObjectsManipulator(target_object, std::vector<ObjectsModels::PickUpObject>(), std::vector<ObjectsModels::PickUpObstacle>())
    {}

    /* PUBLIC PROGRAM FUNCTIONS */
    /* This function executes the picking-up routine for target_object (returns true if succeeded) */
    bool pick_up_target_object();  
    /* This function executes the placing routine for target_object (returns true if succeeded) */
    bool place_target_object();  
    
	
private:
    ros::NodeHandle nh; 
    ObjectsModels::PickUpObject target_object;
    std::vector<ObjectsModels::PickUpObject> detected_objects;
    std::vector<ObjectsModels::PickUpObstacle> detected_obstacles;

    std::vector<moveit_msgs::CollisionObject> collision_objects;

    moveit::planning_interface::MoveGroupInterface group_arm_torso;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    ros::ServiceClient attachClient;
	ros::ServiceClient detachClient;


    
    /* PRIVATE PROGRAM FUNCTIONS */

    void create_collison_objects();
    void remove_collison_objects();

    void move_arm(geometry_msgs::PoseStamped goal_pose);
    void move_torso(geometry_msgs::PoseStamped goal_pose);
    
    void close_gripper();
    void open_gripper();
    
    void attachObjects(const std::string& model1, const std::string& link1,
                   const std::string& model2, const std::string& link2);
	void detachObjects(const std::string& model1, const std::string& link1,
                   const std::string& model2, const std::string& link2);

};



#endif // OBJECTS_DETECTOR_H
