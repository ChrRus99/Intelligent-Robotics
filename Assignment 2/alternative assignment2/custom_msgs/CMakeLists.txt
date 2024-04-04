cmake_minimum_required(VERSION 3.0.2)
project(custom_msgs)

################################################
## ****************************************** ##
################################################

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation
	actionlib
  	actionlib_msgs
  	geometry_msgs
  	control_msgs
  	sensor_msgs
	apriltag_ros
	trajectory_msgs
	tf2_ros
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Writing an action server
add_action_files( 
	DIRECTORY action
	FILES 
	Navigation.action
	ObjectManipulation.action
)

add_service_files( 
	FILES detected_objects.srv
)

## Generate messages in the 'msg' folder
add_message_files(
	FILES 
	PickUpObject.msg
	PickUpObstacle.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
	std_msgs
	actionlib_msgs
	geometry_msgs

)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS 
	roscpp
	message_runtime
	actionlib
	actionlib_msgs
	geometry_msgs
	control_msgs
	trajectory_msgs
)


###########
## Build ##
###########


#############
## Install ##
#############


#############
## Testing ##
#############
