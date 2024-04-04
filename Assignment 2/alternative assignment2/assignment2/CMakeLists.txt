cmake_minimum_required(VERSION 3.0.2)
project(assignment2)

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
	moveit_core
  	moveit_ros_planning_interface
	custom_msgs
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

## Generate services in the 'srv' folder
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
	custom_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
	include
	${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include/util
	${CMAKE_CURRENT_SOURCE_DIR}/include/models
    ${CMAKE_CURRENT_SOURCE_DIR}/include/operations
    ${CMAKE_CURRENT_SOURCE_DIR}/include/nodes
)

## Declare a C++ library
add_library(Point
	src/util/Point.cpp
)
add_library(Circle
	src/util/Circle.cpp
	src/util/Point.cpp
)
add_library(Tree
	src/util/Tree.cpp
	src/models/ObjectsModels.cpp
	src/util/Point.cpp
)
add_library(PointParser
	src/util/PointParser.cpp
	src/util/Point.cpp
	src/models/ObjectsModels.cpp
)
add_library(ComputerVisionUtils
	src/util/ComputerVisionUtils.cpp
)
add_library(ObjectsModels
	src/models/ObjectsModels.cpp
	src/util/Point.cpp
)
add_library(NavigationModels
	src/models/NavigationModels.cpp
	src/models/ObjectsModels.cpp
	src/util/Point.cpp
)
add_library(TiagoHeadController
	src/operations/TiagoHeadController.cpp
)
add_library(ObstaclesDetector
    src/operations/ObstaclesDetector.cpp
	src/util/Circle.cpp
	src/util/Point.cpp
)
add_library(ObjectsDetector
	src/operations/ObjectsDetector.cpp
	src/operations/TiagoHeadController.cpp
	src/models/ObjectsModels.cpp
	src/util/Point.cpp
)
add_library(ObjectsManipulator
	src/operations/ObjectsManipulator.cpp
	src/models/ObjectsModels.cpp
	src/util/PointParser.cpp
	src/util/Point.cpp
)
add_library(DeliveryTablesDetector
	src/operations/DeliveryTablesDetector.cpp
	src/operations/ObstaclesDetector.cpp
	src/models/ObjectsModels.cpp
	src/util/ComputerVisionUtils.cpp
	src/util/Point.cpp
)
add_library(NavigationActionClient 
    src/nodes/NavigationActionClient.cpp
	src/util/PointParser.cpp
	src/util/Point.cpp
)
add_library(NavigationActionServer
	src/nodes/NavigationActionServer.cpp
	src/operations/ObstaclesDetector.cpp
	src/util/Circle.cpp
	src/util/Point.cpp
)
add_library(NodeBServiceServer
	src/nodes/NodeBServiceServer.cpp
	src/operations/ObjectsDetector.cpp
	src/operations/TiagoHeadController.cpp
	src/models/ObjectsModels.cpp
	src/util/PointParser.cpp
	src/util/Point.cpp
)
add_library(NodeCActionClient
	src/nodes/NodeCActionClient.cpp
	src/models/ObjectsModels.cpp
	src/util/PointParser.cpp
	src/util/Point.cpp
)
add_library(NodeCActionServer
	src/nodes/NodeCActionServer.cpp
	src/operations/ObjectsManipulator.cpp
	src/models/ObjectsModels.cpp
	src/util/PointParser.cpp
	src/util/Point.cpp
)
add_library(NodeA
	src/nodes/NodeA.cpp
	src/nodes/NavigationActionClient.cpp
	src/nodes/NodeCActionClient.cpp
	src/models/NavigationModels.cpp
	src/models/ObjectsModels.cpp
	src/operations/DeliveryTablesDetector.cpp
	src/operations/ObstaclesDetector.cpp
	src/operations/TiagoHeadController.cpp
	src/util/ComputerVisionUtils.cpp
	src/util/Circle.cpp
	src/util/PointParser.cpp
	src/util/Point.cpp
	src/util/Tree.cpp
)

# Specify include directories for libraries and executables
target_include_directories(Point PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/util)
target_include_directories(Circle PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/util)
target_include_directories(Tree PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/util)
target_include_directories(PointParser PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/util)
target_include_directories(ComputerVisionUtils PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/util)
target_include_directories(ObjectsModels PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/models)
target_include_directories(NavigationModels PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/models)
target_include_directories(TiagoHeadController PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/operations)
target_include_directories(ObstaclesDetector PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/operations)
target_include_directories(ObjectsDetector PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/operations)
target_include_directories(ObjectsManipulator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/operations)
target_include_directories(DeliveryTablesDetector PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/operations)
target_include_directories(NavigationActionClient PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/nodes)
target_include_directories(NavigationActionServer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/nodes)
target_include_directories(NodeBServiceServer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/nodes)
target_include_directories(NodeCActionClient PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/nodes)
target_include_directories(NodeCActionServer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/nodes)
target_include_directories(NodeA PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/nodes)

## Declare a C++ executable
# Building nodes
add_executable(main_node_navigation_action_server src/main_node_navigation_action_server.cpp)
add_executable(main_node_A src/main_node_A.cpp)
add_executable(main_node_B_service_server src/main_node_B_service_server.cpp)
add_executable(main_node_C_action_server src/main_node_C_action_server.cpp)

## Specify libraries to link each executable against
# Link the library to the executables
target_link_libraries(main_node_navigation_action_server
	NavigationActionServer	
	${catkin_LIBRARIES}
)
target_link_libraries(main_node_B_service_server
	NodeBServiceServer	
	${catkin_LIBRARIES}
)
target_link_libraries(main_node_C_action_server
	NodeCActionServer	
	${catkin_LIBRARIES}
)
target_link_libraries(main_node_A
	NodeA	
	NavigationActionClient
	NodeCActionClient
	${catkin_LIBRARIES}
)

## Add cmake target dependencies of the library
add_dependencies(main_node_navigation_action_server ${assignment2_EXPORTED_TARGETS})
add_dependencies(main_node_B_service_server ${assignment2_EXPORTED_TARGETS})
add_dependencies(main_node_C_action_server ${assignment2_EXPORTED_TARGETS})
add_dependencies(main_node_A ${assignment2_EXPORTED_TARGETS})


#############
## Install ##
#############


#############
## Testing ##
#############


