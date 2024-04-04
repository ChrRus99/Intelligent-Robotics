cmake_minimum_required(VERSION 3.0.2)
project(exercise4)

################################################
## ****************************************** ##
################################################

find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package()


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(PeoplePositionCalculator
  src/PeoplePositionCalculator.cpp
)

## Declare a C++ executable
# Building nodes
add_executable(robot_bags_handler src/robot_bags_handler.cpp)

## Specify libraries to link each executable against
# Link the library to the executables
target_link_libraries(robot_bags_handler 
	PeoplePositionCalculator
	${catkin_LIBRARIES}
)

## Add cmake target dependencies of the library



#############
## Install ##
#############


#############
## Testing ##
#############


