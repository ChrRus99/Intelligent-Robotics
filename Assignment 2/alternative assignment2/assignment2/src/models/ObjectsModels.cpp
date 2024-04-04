#include "models/ObjectsModels.h"
#include <sstream>
#include <cstring>

using namespace std;
using namespace Point;

namespace ObjectsModels {
    /* PICK_UP_OBJECT */
    PickUpObject::PickUpObject(PickUpObject::ObjectType object_type, const Position& object_position) : PickUpObject(
        object_type, to_object_id(object_type), 
        object_position,
        get_pick_up_object_color(object_type)
    ) {}

    PickUpObject::PickUpObject(int object_id, const Position& object_position) : PickUpObject(
        to_object_type(object_id),
        object_id, object_position,
        get_pick_up_object_color(to_object_type(object_id))
    ) {}

    ostream& operator<<(ostream& os, const PickUpObject& object) 
    {
        os << "{Name: " << to_string(object.get_object_type()) 
           << ", ID: " << object.get_object_id();

        if(object.get_object_position() != INVALID_POSITION)
            os << ", Position: " << to_string(object.get_object_position());

        os << "}";

        return os;
    }

    int to_object_id(PickUpObject::ObjectType object_type)
    {
        return static_cast<int>(object_type) + 1;
    }

    PickUpObject::ObjectType to_object_type(int object_id)
    {
        return static_cast<PickUpObject::ObjectType>(object_id - 1);
    }

    Pose get_navigation_pick_up_pose(const PickUpObject& object)
    {
        switch (object.get_object_type()) {
            case PickUpObject::ObjectType::BLUE_HEXAGON:
                return Pose(8.0, -2.1, -90.0);  // oppure metti (8.1, -2.1, -100.0) e trova di sicuro il blu, ma non l'ostacolo   (8.0, -2.0, -90.0)
            case PickUpObject::ObjectType::GREEN_TRIANGLE:
                return Pose(7.6, -4.1, 85.0);
            case PickUpObject::ObjectType::RED_CUBE:
                return Pose(7.5, -2.0, -80.0);
            default:
                return INVALID_POSE;
        }
    }

    Color get_pick_up_object_color(PickUpObject::ObjectType object_type)
    {
        return static_cast<Color>(object_type);
    }
    
    PickUpObject::ObjectType get_corresponding_pick_up_object_type(Color color)
    {
        return static_cast<PickUpObject::ObjectType>(color);
    }

    const char* to_string(PickUpObject::ObjectType object_type) 
    {
        switch (object_type) {
            case PickUpObject::ObjectType::BLUE_HEXAGON:
                return "blue_hexagon";
            case PickUpObject::ObjectType::GREEN_TRIANGLE:
                return "green_triangle";
            case PickUpObject::ObjectType::RED_CUBE:
                return "red_cube";
            default:
                return "unknown_type";
        }
    }

    const char* to_string(const PickUpObject& object) 
    {
        std::ostringstream oss;
        oss << "{Name: " << to_string(object.get_object_type())
            << ", ID: " << object.get_object_id();

        if(object.get_object_position() != INVALID_POSITION)
            oss << ", Position: " << to_string(object.get_object_position());

        oss << "}";

        // Allocate memory for the C-style string and copy the content
        char* result = strdup(oss.str().c_str());

        return result;
    }


    /* PICK_UP_OBSTACLES ENUM */
    PickUpObstacle::PickUpObstacle(PickUpObstacle::ObstacleType obstacle_type, Position obstacle_position)
        : PickUpObstacle(obstacle_type, to_obstacle_id(obstacle_type), obstacle_position) {}

    PickUpObstacle::PickUpObstacle(int obstacle_id, Position obstacle_position)
        : PickUpObstacle(to_obstacle_type(obstacle_id), obstacle_id, obstacle_position) {}

    ostream& operator<<(ostream& os, const PickUpObstacle& obstacle) {
        os << "{Name: " << to_string(obstacle.get_obstacle_type()) 
        << ", ID: " << obstacle.get_obstacle_id();

        if(obstacle.get_obstacle_position() != INVALID_POSITION)
            os << ", Position: " << to_string(obstacle.get_obstacle_position());

        os << "}";

        return os;
    }

    int to_obstacle_id(PickUpObstacle::ObstacleType obstacle_type)
    {
        return PickUpObject::NUM_OBJECT_TYPES + static_cast<int>(obstacle_type) + 1;
    }

    PickUpObstacle::ObstacleType to_obstacle_type(int obstacle_id)
    {
        return static_cast<PickUpObstacle::ObstacleType>(obstacle_id - PickUpObject::NUM_OBJECT_TYPES - 1);
    }

    const char* to_string(PickUpObstacle::ObstacleType obstacle_type) {
        switch (obstacle_type) {
            case PickUpObstacle::ObstacleType::GOLD_OBS_0:
                return "gold_obstacle_0";
            case PickUpObstacle::ObstacleType::GOLD_OBS_1:
                return "gold_obstacle_1";
            case PickUpObstacle::ObstacleType::GOLD_OBS_2:
                return "gold_obstacle_2";
            case PickUpObstacle::ObstacleType::GOLD_OBS_3:
                return "gold_obstacle_3";
            default:
                return "unknown_type";
        }
    }

    const char* to_string(const PickUpObstacle& obstacle) {
        std::ostringstream oss;
        oss << "{Name: " << to_string(obstacle.get_obstacle_type())
            << ", ID: " << obstacle.get_obstacle_id();
        
        if(obstacle.get_obstacle_position() != INVALID_POSITION)
            oss << ", Position: " << to_string(obstacle.get_obstacle_position());

        oss << "}";

        // Allocate memory for the C-style string and copy the content
        char* result = strdup(oss.str().c_str());

        return result;
    }


    /* DELIVERY_TABLE*/
    DeliveryTable::DeliveryTable(TableType table_type, Point::Pose pose, PickUpObject::ObjectType object_type, Color color)
        : table_type(table_type), pose(pose), object_type(object_type), color(color) 
    {
        // If the pose is invalid assume the default one  
        this->pose = (pose != INVALID_POSE) ? pose : get_navigation_delivery_pose(object_type); 
    }

    DeliveryTable::DeliveryTable(TableType table_type, Pose pose) : DeliveryTable(
        table_type,
        pose,
        get_corresponding_pick_up_object_type(table_type),
        get_delivery_table_color(table_type)
    ) {}
    
    DeliveryTable::DeliveryTable(PickUpObject::ObjectType object_type, Pose pose) : DeliveryTable(
        get_corresponding_delivery_table_type(object_type),
        pose,
        object_type,
        get_delivery_table_color(get_corresponding_delivery_table_type(object_type))
    ) {}
    
    DeliveryTable::DeliveryTable(Color color, Pose pose) : DeliveryTable(
        get_corresponding_delivery_table_type(color),
        pose,
        get_corresponding_pick_up_object_type(get_corresponding_delivery_table_type(color)),
        color
    ) {}

    DeliveryTable::DeliveryTable(Point::Pose pose) : DeliveryTable(
        TableType::NONE,
        pose,
        PickUpObject::ObjectType::NONE,
        Color::NONE
    ) {}

    DeliveryTable::TableType get_corresponding_delivery_table_type(PickUpObject::ObjectType object_type)
    {
        return static_cast<DeliveryTable::TableType>(object_type);
    }

    Pose get_navigation_delivery_pose(PickUpObject::ObjectType object_type)
    {
        switch(get_corresponding_delivery_table_type(object_type)) {
            case DeliveryTable::TableType::BLUE_TABLE:
                return Pose(12.50, -0.92, 90.0);
            case DeliveryTable::TableType::GREEN_TABLE:
                return Pose(11.50, -0.92, 90.0);
            case DeliveryTable::TableType::RED_TABLE:
                return Pose(10.50, -0.92, 90.0);
            default:
                return INVALID_POSE;
        }
    }

    PickUpObject::ObjectType get_corresponding_pick_up_object_type(DeliveryTable::TableType table_type) 
    {
        return static_cast<PickUpObject::ObjectType>(table_type);
    }

    Color get_delivery_table_color(DeliveryTable::TableType table_type) 
    {
        return static_cast<Color>(table_type);
    }

    DeliveryTable::TableType get_corresponding_delivery_table_type(Color color) 
    {
        return static_cast<DeliveryTable::TableType>(color);
    }

    const char* to_string(DeliveryTable::TableType table_type) 
    {
        switch(table_type) {
            case DeliveryTable::TableType::BLUE_TABLE:
                return "Blue table";
            case DeliveryTable::TableType::GREEN_TABLE:
                return "Green table";
            case DeliveryTable::TableType::RED_TABLE:
                return "Red table";
            default:
                return "Unknown table type";
        }
    }

    const char* to_string(const DeliveryTable& delivery_table) 
    {
        std::ostringstream oss;
        oss << "{Name: " << to_string(delivery_table.get_table_type());
        
        if(delivery_table.get_pose() != INVALID_POSE)
            oss << ", Position: " << to_string(delivery_table.get_pose());

        oss << "}";

        // Allocate memory for the C-style string and copy the content
        char* result = strdup(oss.str().c_str());

        return result;
    }
}