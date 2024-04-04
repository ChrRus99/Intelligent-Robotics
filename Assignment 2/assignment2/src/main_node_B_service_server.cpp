#include "nodes/NodeBServiceServer.h"

int main(int argc, char** argv) {
    // Initialize node_A
    ros::init(argc, argv, "node_B_service_server");

    // Instantiate NodeBServiceServer
    NodeBServiceServer service;

    ros::spin();

    return 0;
}
