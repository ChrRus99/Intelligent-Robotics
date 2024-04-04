#ifndef OBJECTS_MODELS_H
#define OBJECTS_MODELS_H

#include "util/Point.h"
#include <limits>
#include <iostream>

namespace ObjectsModels {
    static const Point::Position INVALID_POSITION(
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()
    );
    static const Point::Pose INVALID_POSE(
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()
    );


    /* COLOR ENUM */
    enum class Color
    { 
        BLUE,
        GREEN,
        RED,
        NONE = -1   // Represents a null color
    };


    /* PICK_UP_OBJECT */
    class PickUpObject
    {
    public:
        enum class ObjectType
        {
            BLUE_HEXAGON,
            GREEN_TRIANGLE,
            RED_CUBE,
            NONE = -1   // Represents a null object_type
        };

        static const int NUM_OBJECT_TYPES = 3;

        PickUpObject(ObjectType object_type, const Point::Position& object_position = INVALID_POSITION); 
        PickUpObject(int object_id, const Point::Position& object_position = INVALID_POSITION);


        /* PUBLIC FUNCTIONS */
        inline void set_object_position(const Point::Position& object_position) { this->object_position = object_position; }
        inline int get_object_id() const { return object_id; }
        inline ObjectType get_object_type() const { return object_type; }
        inline Point::Position get_object_position() const { return object_position; }
        inline Color get_color() const { return color; }

        /* Overloading of the operator<< */
        friend std::ostream& operator<<(std::ostream& os, const PickUpObject& obj);

    private:
        ObjectType object_type;
        int object_id;
        Point::Position object_position;
        Color color;

        PickUpObject(ObjectType object_type, int object_id, const Point::Position& object_position, Color color)
            : object_type(object_type), object_id(object_id), object_position(object_position), color(color) {}
    };

    /* HELPER FUNCTIONS */
    /* This function returns the default object_id for the given object_type */
    int to_object_id(PickUpObject::ObjectType object_type);
    /* This function returns the default object_type for the given object_id */
    PickUpObject::ObjectType to_object_type(int object_id);
    /* This function returns the default pick-up pose for the robot to pick-up the given object */
    Point::Pose get_navigation_pick_up_pose(const PickUpObject& object);
    /* This function returns the color of the given pick-up object type*/
    Color get_pick_up_object_color(PickUpObject::ObjectType object_type);
    /* This function returns the type of pick-up object corresponding the given color */
    PickUpObject::ObjectType get_corresponding_pick_up_object_type(Color color);
    /* This function returns the default name of the given object_type */
    const char* to_string(PickUpObject::ObjectType object_type);
    /* This function substitutes the operator<<, which does not work for ROS_INFO */
    const char* to_string(const PickUpObject& object);
    

    /* PICK_UP_OBSTACLES */
    class PickUpObstacle 
    {
    public:
        enum class ObstacleType 
        {
            GOLD_OBS_0,
            GOLD_OBS_1,
            GOLD_OBS_2,
            GOLD_OBS_3,
        };

        PickUpObstacle(ObstacleType obstacle_type, Point::Position obstacle_position = INVALID_POSITION);
        PickUpObstacle(int obstacle_id, Point::Position obstacle_position = INVALID_POSITION);

        /* PUBLIC FUNCTIONS */
        inline void set_obstacle_position(const Point::Position& obstacle_position) { this->obstacle_position = obstacle_position; }
        inline int get_obstacle_id() const { return obstacle_id; }
        inline ObstacleType get_obstacle_type() const { return obstacle_type; }
        inline Point::Position get_obstacle_position() const { return obstacle_position; }   

        /* Overloading of the operator<< */
        friend std::ostream& operator<<(std::ostream& os, const PickUpObstacle& obstacle);

    private:
        ObstacleType obstacle_type;
        int obstacle_id;
        Point::Position obstacle_position;

        PickUpObstacle(ObstacleType obstacle_type, int obstacle_id, const Point::Position& obstacle_position)
            : obstacle_type(obstacle_type), obstacle_id(obstacle_id), obstacle_position(obstacle_position) {}
    };

    /* HELPER FUNCTIONS */
    /* This function returns the default obstacle_id for the given obstacle_type */
    int to_obstacle_id(PickUpObstacle::ObstacleType obstacle_type);
    /* This function returns the default obstacle_type for the given obstacle_id */
    PickUpObstacle::ObstacleType to_obstacle_type(int obstacle_id);
    /* This function returns the default name of the given obstacle_type */
    const char* to_string(PickUpObstacle::ObstacleType obstacle_type);
    /* This function substitutes the operator<<, which does not work for ROS_INFO */
    const char* to_string(const PickUpObstacle& obstacle);


    /* DELIVERY_TABLE */
    class DeliveryTable
    {
    public:
        enum class TableType
        {
            BLUE_TABLE,
            GREEN_TABLE,
            RED_TABLE,
            NONE = -1   // Represents a null table_type
        };

        DeliveryTable(TableType table_type, Point::Pose pose = INVALID_POSE);
        DeliveryTable(PickUpObject::ObjectType object_type, Point::Pose pose = INVALID_POSE);
        DeliveryTable(Color color, Point::Pose pose = INVALID_POSE);
        DeliveryTable(Point::Pose pose);

        /* PUBLIC FUNCTIONS */
        inline void set_pose(Point::Pose pose) { this->pose = pose; }
        // Note: the setter functions below modify consistently all the properties of this delivery table instance, 
        // preserving only the current pose. They are meant to be used in combination with the 'DeliveryTable(Point::Pose pose)'
        // constructor, for cases in which the pose of the delivery table is known, but its type is not known yet.
        inline void set_table_type(TableType table_type) { *this = {table_type, pose}; }
        inline void set_object_type(PickUpObject::ObjectType object_type) { *this = {object_type, pose}; }
        inline void set_color(Color color) { *this = {color, pose}; }
        inline TableType get_table_type() const { return table_type; }
        inline Point::Pose get_pose() const { return pose; }
        inline PickUpObject::ObjectType get_object_type() const { return object_type; }
        inline Color get_color() const { return color; }
    
    private:
        TableType table_type;
        Point::Pose pose;
        PickUpObject::ObjectType object_type;
        Color color;

        DeliveryTable(TableType table_type, Point::Pose pose, PickUpObject::ObjectType object_type, Color color);
    };

    /* HELPER FUNCTIONS */
    /* This function returns the type of delivery table corresponding to the given pick-up object type */
    DeliveryTable::TableType get_corresponding_delivery_table_type(PickUpObject::ObjectType object_type);
    /* This function returns the default pose for the robot to deliver the given object */
    Point::Pose get_navigation_delivery_pose(PickUpObject::ObjectType object_type);
    /* This function returns the type of pick-up object that can be placed obove the given delivery table type*/
    PickUpObject::ObjectType get_corresponding_pick_up_object_type(DeliveryTable::TableType table_type);
    /* This function returns the color of the given delivery table type*/
    Color get_delivery_table_color(DeliveryTable::TableType table_type);
    /* This function returns the type of delivery table corresponding the given color */
    DeliveryTable::TableType get_corresponding_delivery_table_type(Color color);
    /* This function returns the default name of the given table_type */
    const char* to_string(DeliveryTable::TableType table_type);
    /* This function substitutes the operator<<, which does not work for ROS_INFO */
    const char* to_string(const DeliveryTable& delivery_table);   
}

#endif // OBJECTS_MODELS_H