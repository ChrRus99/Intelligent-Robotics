cmake_minimum_required(VERSION 3.0.2)
project(assignment1)

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
  	sensor_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Writing an action server
add_action_files( 
	DIRECTORY action
	FILES Navigation.action
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
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(Point
	src/util/Point.cpp
)
add_library(Circle
	src/util/Circle.cpp
	src/util/Point.cpp
)
add_library(ObstaclesDetector
    src/ObstaclesDetector.cpp
	src/util/Circle.cpp
	src/util/Point.cpp
)
add_library(NarrowCorridorControlLaw
    src/NarrowCorridorControlLaw.cpp
)
add_library(NavigationClient 
    src/NavigationClient.cpp
)
add_library(NavigationServer
	src/NavigationServer.cpp
	src/NarrowCorridorControlLaw.cpp
	src/ObstaclesDetector.cpp
	src/util/Circle.cpp
	src/util/Point.cpp
)

# Specify include directories for libraries and executables
target_include_directories(Point PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/util)
target_include_directories(Circle PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/util)
target_include_directories(NarrowCorridorControlLaw PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(ObstaclesDetector PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(NavigationClient PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(NavigationServer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

## Declare a C++ executable
# Building nodes
add_executable(main_action_client_node src/main_action_client_node.cpp)
add_executable(main_action_server_node src/main_action_server_node.cpp)

## Specify libraries to link each executable against
# Link the library to the executables
target_link_libraries(main_action_client_node 
	NavigationClient
	${catkin_LIBRARIES}
)

target_link_libraries(main_action_server_node
	Point
	ObstaclesDetector
	NavigationServer	
	${catkin_LIBRARIES}
)

## Add cmake target dependencies of the library
add_dependencies(NavigationServer ${catkin_EXPORTED_TARGETS})

add_dependencies(main_action_client_node ${assignment1_EXPORTED_TARGETS})
add_dependencies(main_action_server_node ${assignment1_EXPORTED_TARGETS})


#############
## Install ##
#############


#############
## Testing ##
#############


