#include "operations/ObjectsManipulator.h"

using namespace std;
using namespace Point;
using namespace ObjectsModels;

/* PUBLIC PROGRAM FUNCTIONS */
bool ObjectsManipulator::pick_up_target_object()
{
    ROS_INFO("Object manipulator ----------> TARGET OBJECT: %s", to_string(target_object));

    for(const auto& object : detected_objects) {
        ROS_INFO("Object manipulator ----------> OBJECT: %s", to_string(object));
    }

    for(const auto& obstacle : detected_obstacles) {
        ROS_INFO("Object manipulator ----------> OBSTACLE: %s", to_string(obstacle));
    }

    create_collison_objects();
    
    geometry_msgs::PoseStamped home_position = group_arm_torso.getCurrentPose();
    geometry_msgs::PoseStamped safe_position;
    geometry_msgs::PoseStamped above_pick_position;
    geometry_msgs::PoseStamped pick_position;
	
	if (target_object.get_object_id() == 1)
    {

        float x= detected_objects[0].get_object_position().x;
        float y= detected_objects[0].get_object_position().y;
		float z= detected_objects[0].get_object_position().z;
		
        safe_position.header.frame_id = "base_footprint";
        safe_position.pose.position.x = 0.3;
        safe_position.pose.position.y = 0.8;
        safe_position.pose.position.z = 1.0;
        safe_position.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.5, 0.0, 1.5);

		// Pick from the side
		/*above_pick_position.header.frame_id = "base_footprint";
        above_pick_position.pose.position.x = x;
        above_pick_position.pose.position.y = y + 0.2;
        above_pick_position.pose.position.z = 1.0;
        above_pick_position.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.5, 0.0, -1.5);

        pick_position.header.frame_id = "base_footprint";
        pick_position.pose.position.x = x;
        pick_position.pose.position.y = y + 0.2;
        pick_position.pose.position.z = 0.85;
        pick_position.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.5, 0.0, -1.5);*/

		// Pick from above
        above_pick_position.header.frame_id = "base_footprint";
        above_pick_position.pose.position.x = x;
        above_pick_position.pose.position.y = y;
        above_pick_position.pose.position.z = z + 0.5;
        above_pick_position.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 1.5, 0.0);

        pick_position.header.frame_id = "base_footprint";
        pick_position.pose.position.x = x;
        pick_position.pose.position.y = y;
        pick_position.pose.position.z = z + 0.2;
        pick_position.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 1.5, 0.0);
    }

    if ( target_object.get_object_id() == 2)
    {

        float x= detected_objects[0].get_object_position().x;
        float y= detected_objects[0].get_object_position().y;
		float z= detected_objects[0].get_object_position().z;

        safe_position.header.frame_id = "base_footprint";
        safe_position.pose.position.x = 0.3;
        safe_position.pose.position.y = 0.8;
        safe_position.pose.position.z = 1.2;//1.0
        safe_position.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.5, 0.0, 1.5);

        above_pick_position.header.frame_id = "base_footprint";
        above_pick_position.pose.position.x = x;
        above_pick_position.pose.position.y = y;
        above_pick_position.pose.position.z = z + 0.5;
        above_pick_position.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 1.5, 0.0);

        pick_position.header.frame_id = "base_footprint";
        pick_position.pose.position.x = x;
        pick_position.pose.position.y = y;
        pick_position.pose.position.z = z + 0.3;
        pick_position.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 1.5, 0.0);

    }

    if (target_object.get_object_id() == 3)
    {
        float x= detected_objects[0].get_object_position().x;
        float y= detected_objects[0].get_object_position().y;
		float z= detected_objects[0].get_object_position().z;

        safe_position.pose.position.x = 0.3;
        safe_position.pose.position.y = 0.8;
        safe_position.pose.position.z = 1.0;
        safe_position.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.5, 0.0, 1.5);

        above_pick_position.header.frame_id = "base_footprint";
        above_pick_position.pose.position.x = x;
        above_pick_position.pose.position.y = y;
        above_pick_position.pose.position.z = z + 0.55;
        above_pick_position.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 1.5, 0.0);

        pick_position.header.frame_id = "base_footprint";
        pick_position.pose.position.x = x;
        pick_position.pose.position.y = y;
        pick_position.pose.position.z = z + 0.3;
        pick_position.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 1.5, 0.0);
    }

    ROS_INFO("----------> SAFE_POSITION:");
    ROS_INFO("  Position (x, y, z): %f, %f, %f", safe_position.pose.position.x,
                                           safe_position.pose.position.y,
                                           safe_position.pose.position.z);
    ROS_INFO("  Orientation (x, y, z, w): %f, %f, %f, %f", safe_position.pose.orientation.x,
                                                      safe_position.pose.orientation.y,
                                                      safe_position.pose.orientation.z,
                                                      safe_position.pose.orientation.w);

    ROS_INFO("----------> PICK_POSITION:");
    ROS_INFO("  Position (x, y, z): %f, %f, %f", pick_position.pose.position.x,
                                           pick_position.pose.position.y,
                                           pick_position.pose.position.z);
    ROS_INFO("  Orientation (x, y, z, w): %f, %f, %f, %f", pick_position.pose.orientation.x,
                                                      pick_position.pose.orientation.y,
                                                      pick_position.pose.orientation.z,
                                                      pick_position.pose.orientation.w);


    
    
    move_arm(safe_position);
    ROS_INFO ("----------> SAFE_POSITION REACHED");

    move_arm(above_pick_position);
    ROS_INFO ("----------> ABOVE_PICK_POSITION REACHED");
	
	open_gripper();
	
    //REMOVE TARGET OBJECT FROM COLLISION OBJECT
    std::vector<std::string> object_to_remove;
    //ROS_INFO("Object ID: %d", target_object.get_object_id());
    string id_to_remove = to_string(target_object.get_object_id());
    //ROS_INFO("########## ID: %s", id_to_remove.c_str());
    object_to_remove.push_back(id_to_remove.c_str());
    planning_scene_interface.removeCollisionObjects(object_to_remove);

    move_arm(pick_position);
    ROS_INFO ("----------> PICK_POSITION REACHED");

    close_gripper();
	
	// Attach virtually the object to the robot gripper
	if (target_object.get_object_id() == 1) {
		attachObjects("tiago", "arm_7_link", "Hexagon", "Hexagon_link");
	}
	else if (target_object.get_object_id() == 2) {
		attachObjects("tiago", "arm_7_link", "Triangle", "Triangle_link");
	}
	else if (target_object.get_object_id() == 3) {
		attachObjects("tiago", "arm_7_link", "cube", "cube_link");
	}
	else {
		ROS_INFO ("----------> Error on attacher: object not identified");
	}


    move_arm(above_pick_position);
    ROS_INFO ("----------> ABOVE_PICK_POSITION REACHED");
    move_arm(safe_position);
    ROS_INFO ("----------> SAFE_POSITION REACHED");
    move_arm(home_position);
    ROS_INFO ("----------> HOME_POSITION REACHED");
    

    remove_collison_objects();

    return true;
} 

bool ObjectsManipulator::place_target_object()
{

    ROS_INFO("Object manipulator ----------> TARGET OBJECT: %s", to_string(target_object));


    geometry_msgs::PoseStamped home_place_position = group_arm_torso.getCurrentPose();
    geometry_msgs::PoseStamped safe_place_position;
    geometry_msgs::PoseStamped place_position;

    if ( target_object.get_object_id() == 1)
    {
        ROS_INFO ("----------> ENTRA 2");
        safe_place_position.header.frame_id = "base_footprint";
        safe_place_position.pose.position.x = 0.7;
        safe_place_position.pose.position.y = 0.15;
        safe_place_position.pose.position.z = 1.2;
        safe_place_position.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.5, 1.5, -1.5);//(1.5, 0.0, -1.5);//(1.5, 1.5, -1.5);

        place_position.header.frame_id = "base_footprint";
        place_position.pose.position.x = 0.7;
        place_position.pose.position.y = 0.15;
        place_position.pose.position.z = 1.05;		
        place_position.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 1.5, 0.0);//(1.5, 0.0, -1.5);//(1.5, 1.5, -1.5);

    }

    if ( target_object.get_object_id() == 2)
    {
        ROS_INFO ("----------> ENTRA 2");
        safe_place_position.header.frame_id = "base_footprint";
        safe_place_position.pose.position.x = 0.75;
        safe_place_position.pose.position.y = 0.15;
        safe_place_position.pose.position.z = 1.2;
        safe_place_position.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.5, 1.5, -1.5);

        place_position.header.frame_id = "base_footprint";
        place_position.pose.position.x = 0.75;
        place_position.pose.position.y = 0.15;
        place_position.pose.position.z = 0.94;		
        place_position.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 1.5, 0.0);//(1.5, 1.5, -1.5);

    }

    if ( target_object.get_object_id() == 3)
    {
        ROS_INFO ("----------> ENTRA 2");
        safe_place_position.header.frame_id = "base_footprint";
        safe_place_position.pose.position.x = 0.73;
        safe_place_position.pose.position.y = 0.05;
        safe_place_position.pose.position.z = 1.2;
        safe_place_position.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.5, 1.5, -1.5);

        place_position.header.frame_id = "base_footprint";
        place_position.pose.position.x = 0.73;
        place_position.pose.position.y = 0.05;
        place_position.pose.position.z = 1.0;//0.94;		
        place_position.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 1.5, 0.0);//(1.5, 1.5, -1.5);

    }


    
    //ADDING PLACE_TABLE AS COLLISON OBJECT
    moveit_msgs::CollisionObject object4;
    object4.header.frame_id = "base_footprint";
    object4.id = "place_table";
    object4.primitives.resize(1);
    object4.primitives[0].type = object4.primitives[0].CYLINDER;
    object4.primitives[0].dimensions.resize(2);
    object4.primitives[0].dimensions[0] = 0.69; // Lunghezza
    object4.primitives[0].dimensions[1] = 0.23; // Larghezza
    object4.primitive_poses.resize(1);
    ROS_INFO ("----------> ################### %d", target_object.get_object_id());
    if (target_object.get_object_id() == 1)
    {
        object4.primitive_poses[0].position.x = 0.6906;
        object4.primitive_poses[0].position.y = 0.0913;
        ROS_INFO ("----------> ENTRA 1");
    }
    if (target_object.get_object_id() == 2)
    {
        object4.primitive_poses[0].position.x = 0.7616;
        object4.primitive_poses[0].position.y = 0.1904;
        ROS_INFO ("----------> ENTRA 1");

    }
    if (target_object.get_object_id() == 3)
    {
        object4.primitive_poses[0].position.x = 0.7564;
        object4.primitive_poses[0].position.y = 0.0552;
        ROS_INFO ("----------> ENTRA 1");

    }
    
    object4.primitive_poses[0].position.z = 0.345;
    object4.operation = object4.ADD;
    collision_objects.push_back(object4);
    
    planning_scene_interface.applyCollisionObjects(collision_objects);

    //START MOVING ROBOTIC ARM
    move_arm(safe_place_position);
    ROS_INFO ("----------> SAFE_PLACE_POSITION REACHED");

    move_arm(place_position);
    ROS_INFO ("----------> PLACE_POSITION REACHED");

    open_gripper();

	// Detach virtually the object from the robot gripper
	if (target_object.get_object_id() == 1) {
    	detachObjects("tiago", "arm_7_link", "Hexagon", "Hexagon_link");
	}
	else if (target_object.get_object_id() == 2) {
		detachObjects("tiago", "arm_7_link", "Triangle", "Triangle_link");
	}
	else if (target_object.get_object_id() == 3) {
    	detachObjects("tiago", "arm_7_link", "cube", "cube_link");
	}
	else {
		ROS_INFO ("----------> Error on detacher: object not identified");
	}

    move_arm(safe_place_position);
    ROS_INFO ("----------> SAFE_PLACE_POSITION REACHED 2");

    move_arm(home_place_position);
    ROS_INFO ("----------> HOME_PLACE_POSITION REACHED");
    
    return true;
} 

void ObjectsManipulator::move_arm(geometry_msgs::PoseStamped goal_pose)
{
    //ros::AsyncSpinner spinner(1);
    //spinner.start();

    std::vector<std::string> torso_arm_joint_names;    

    group_arm_torso.allowReplanning(true);
    group_arm_torso.setNumPlanningAttempts(50);
    //choose planner
    //group_arm_torso.setPlannerId("SBLkConfigDefault");
    group_arm_torso.setPoseReferenceFrame("base_footprint");
    group_arm_torso.setPoseTarget(goal_pose);

    ROS_INFO_STREAM("Planning to move " << group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " << 
                    group_arm_torso.getPlanningFrame());
    //group_arm_torso.setStartStateToCurrentState();
    //group_arm_torso.setMaxVelocityScalingFactor(1.0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //set maximum time to find a plan
    group_arm_torso.setPlanningTime(5.0);
    
    // VALORE DI TOLLERANZA DELLA GOAL POSITION E ORIENTATION DA SETTARE 
    group_arm_torso.setGoalTolerance(0.05);

    moveit::core::MoveItErrorCode planning_result = group_arm_torso.plan(my_plan);

    if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

        // Execute the plan
        ros::Time start = ros::Time::now();
        group_arm_torso.move();
        ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
        
        // Execute the plan asynchronously
        //group_arm_torso.asyncMove();
        //group_arm_torso.move();

        ROS_INFO("Motion completed");

    } else {
        ROS_ERROR("Failed to plan. MoveIt! error code: %d", planning_result.val);
        //throw std::runtime_error("No plan found");
    }
    //spinner.stop();

}

void ObjectsManipulator::close_gripper()
{
    moveit::planning_interface::MoveGroupInterface group_gripper("gripper");
    // Get the joint names for the gripper
	const std::vector<std::string>& gripper_joint_names = group_gripper.getVariableNames();

    // Set joint values for close the gripper
    std::vector<double> gripper_closed_values = {0.0, 0.0};  //  joint values
    group_gripper.setJointValueTarget(gripper_joint_names, gripper_closed_values);

    // Plan and execute the gripper closing
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    moveit::core::MoveItErrorCode gripper_result = group_gripper.plan(gripper_plan);

    if (gripper_result == moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_INFO_STREAM("Gripper closing plan found in " << gripper_plan.planning_time_ << " seconds");

        // Execute the plan
        ros::Time start = ros::Time::now();
        group_gripper.execute(gripper_plan);
        ROS_INFO_STREAM("Gripper closing duration: " << (ros::Time::now() - start).toSec());
    } else {
        ROS_ERROR("Failed to plan gripper closing. MoveIt! error code: %d", gripper_result.val);
        //throw std::runtime_error("Failed to close gripper");
    }
}

void ObjectsManipulator::open_gripper()
{
    moveit::planning_interface::MoveGroupInterface group_gripper("gripper");
    // Get the joint names for the gripper
	const std::vector<std::string>& gripper_joint_names = group_gripper.getVariableNames();

    // Set joint values for open the gripper
    std::vector<double> gripper_closed_values = {2.0, 2.0};  //  joint values
    group_gripper.setJointValueTarget(gripper_joint_names, gripper_closed_values);

    // Plan and execute the gripper opening
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    moveit::core::MoveItErrorCode gripper_result = group_gripper.plan(gripper_plan);

    if (gripper_result == moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_INFO_STREAM("Gripper opening plan found in " << gripper_plan.planning_time_ << " seconds");

        // Execute the plan
        ros::Time start = ros::Time::now();
        group_gripper.execute(gripper_plan);
        ROS_INFO_STREAM("Gripper opening duration: " << (ros::Time::now() - start).toSec());
    } else {
        ROS_ERROR("Failed to plan gripper opening. MoveIt! error code: %d", gripper_result.val);
        //throw std::runtime_error("Failed to close gripper");
    }
}

void ObjectsManipulator::create_collison_objects()

{
    string id_to_add;
    
    for (size_t i = 0; i < detected_objects.size(); i++)
    {
        //CILINDRO BLU
        if (detected_objects[i].get_object_id() == 1)
        {
            moveit_msgs::CollisionObject object0;
            object0.header.frame_id = "base_footprint";
            id_to_add = to_string(detected_objects[i].get_object_id());
            object0.id = id_to_add.c_str();
            object0.primitives.resize(1);
            object0.primitives[0].type = object0.primitives[0].CYLINDER;
            object0.primitives[0].dimensions.resize(2);
            object0.primitives[0].dimensions[0] = 0.1; //altezza
            object0.primitives[0].dimensions[1] = 0.025; //raggio
            object0.primitive_poses.resize(1);
            Point::Position temp = detected_objects[i].get_object_position();
            object0.primitive_poses[0].position.x = temp.x;
            object0.primitive_poses[0].position.y = temp.y;
            object0.primitive_poses[0].position.z = 0.8803;
            object0.operation = object0.ADD;
            collision_objects.push_back(object0);
            
        }
        //CUBO E TRIANGOLO
        if (detected_objects[i].get_object_id() == 2 || detected_objects[i].get_object_id() == 3)
        {
            moveit_msgs::CollisionObject object0;
            object0.header.frame_id = "base_footprint";
            id_to_add = to_string(detected_objects[i].get_object_id());
            object0.id = id_to_add.c_str();
            object0.primitives.resize(1);
            object0.primitives[0].type = object0.primitives[0].BOX;
            object0.primitives[0].dimensions.resize(3);
            object0.primitives[0].dimensions[0] = 0.05; //altezza
            object0.primitives[0].dimensions[1] = 0.05; //raggio
            object0.primitives[0].dimensions[2] = 0.05; //raggio
            object0.primitive_poses.resize(1);
            Point::Position temp = detected_objects[i].get_object_position();
            object0.primitive_poses[0].position.x = temp.x;
            object0.primitive_poses[0].position.y = temp.y;
            object0.primitive_poses[0].position.z = 0.87;
            object0.operation = object0.ADD;
            collision_objects.push_back(object0);
        }
    }    
    for (size_t i = 0; i < detected_obstacles.size(); i++)
    {
        // CILINDRI OSTACOLI    
        if (detected_obstacles[i].get_obstacle_id() == 4  || detected_obstacles[i].get_obstacle_id() == 5  || detected_obstacles[i].get_obstacle_id() == 6  || detected_obstacles[i].get_obstacle_id() == 7)
        {
            moveit_msgs::CollisionObject object0;
            object0.header.frame_id = "base_footprint";
            id_to_add = to_string(detected_obstacles[i].get_obstacle_id());
            object0.id = id_to_add.c_str();
            object0.primitives.resize(1);
            object0.primitives[0].type = object0.primitives[0].CYLINDER;
            object0.primitives[0].dimensions.resize(2);
            object0.primitives[0].dimensions[0] = 0.2; //altezza
            object0.primitives[0].dimensions[1] = 0.05; //raggio
            object0.primitive_poses.resize(1);
            Point::Position temp = detected_obstacles[i].get_obstacle_position();
            object0.primitive_poses[0].position.x = temp.x;
            object0.primitive_poses[0].position.y = temp.y;
            object0.primitive_poses[0].position.z = 0.8803;
            object0.operation = object0.ADD;
            collision_objects.push_back(object0);
        }
        
    }
    //TAVOLO 
    
    moveit_msgs::CollisionObject object4;
    object4.header.frame_id = "base_footprint";
    object4.id = "box_collision_object4";
    object4.primitives.resize(1);
    object4.primitives[0].type = object4.primitives[0].BOX;
    object4.primitives[0].dimensions.resize(3);
    object4.primitives[0].dimensions[0] = 1;     //0.963; // Lunghezza
    object4.primitives[0].dimensions[1] = 1;     //0.963; // Larghezza
    object4.primitives[0].dimensions[2] = 0.05; // altezza
    object4.primitive_poses.resize(1);
    if (target_object.get_object_id() == 1)
    {
    object4.primitive_poses[0].position.x = 0.88078193;
    object4.primitive_poses[0].position.y = -0.17323725;
    }
    if (target_object.get_object_id() == 2)
    {
    object4.primitive_poses[0].position.x = 1.0802171;
    object4.primitive_poses[0].position.y = -0.2138621;
    }
    if (target_object.get_object_id() == 3)
    {
        object4.primitive_poses[0].position.x = 0.8830580;
        object4.primitive_poses[0].position.y = 0.1525138;
    }
    
    object4.primitive_poses[0].position.z = 0.75;
    object4.operation = object4.ADD;
    collision_objects.push_back(object4);
    
    planning_scene_interface.applyCollisionObjects(collision_objects);
}

void ObjectsManipulator::remove_collison_objects()
{
    std::vector<std::string> collision_object_ids;

    for (size_t i = 0; i < collision_objects.size(); i++)
    {
        collision_object_ids.push_back(collision_objects[i].id);

    }

    // Rimuovi gli oggetti di collisione dalla scena
    planning_scene_interface.removeCollisionObjects(collision_object_ids);

    // Pulisci il vettore dei collision object
    collision_objects.clear();
}

void ObjectsManipulator::move_torso(geometry_msgs::PoseStamped goal_pose)
{
    moveit::planning_interface::MoveGroupInterface group_torso("arm_torso");
    
    // Imposta il riferimento di frame per il gruppo del torso (ad es. "base_link" o "base_footprint")
    group_torso.setPoseReferenceFrame("base_footprint");
    
    // Imposta la pose target per il gruppo del torso
    group_torso.setPoseTarget(goal_pose);

    // Pianifica il movimento del torso
    moveit::planning_interface::MoveGroupInterface::Plan torso_plan;
    moveit::core::MoveItErrorCode torso_planning_result = group_torso.plan(torso_plan);

    if (torso_planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
        // Esegui il movimento del torso
        group_torso.move();
        ROS_INFO("Movimento del torso completato con successo!");
    } else {
        ROS_ERROR("Pianificazione del movimento del torso fallita. Codice errore: %d", torso_planning_result.val);
    }
}

void ObjectsManipulator::attachObjects(const std::string& model1, const std::string& link1,
                   const std::string& model2, const std::string& link2)
{
    gazebo_ros_link_attacher::Attach attachSrv;
    attachSrv.request.model_name_1 = model1;
    attachSrv.request.link_name_1 = link1;
    attachSrv.request.model_name_2 = model2;
    attachSrv.request.link_name_2 = link2;

    if (attachClient.call(attachSrv)) {
        if (attachSrv.response.ok) {
            ROS_INFO("Successfully attached objects");
        }
        else {
            ROS_ERROR("Failed to attach objects");
        }
    }
    else {
        ROS_ERROR("Failed to call attach service");
    }
}

void ObjectsManipulator::detachObjects(const std::string& model1, const std::string& link1,
                   const std::string& model2, const std::string& link2)
{
    gazebo_ros_link_attacher::Attach detachSrv;
    detachSrv.request.model_name_1 = model1;
    detachSrv.request.link_name_1 = link1;
    detachSrv.request.model_name_2 = model2;
    detachSrv.request.link_name_2 = link2;

    if (detachClient.call(detachSrv)) {
        if (detachSrv.response.ok) {
            ROS_INFO("Successfully detached objects");
        }
        else {
            ROS_ERROR("Failed to detach objects");
        }
    }
    else {
        ROS_ERROR("Failed to call detach service");
    }
}
