#ifndef UTIL_POINT_H
#define UTIL_POINT_H

#include <vector>
#include <limits>

namespace Point {
    /* CARTESIAN_POINT AND POLAR_POINT */
    struct CartesianPoint {
        double x, y;

        CartesianPoint(double x, double y) : x(x), y(y) {}
    };

    struct PolarPoint {
        double rho, theta;

        PolarPoint(double rho, double theta) : rho(rho), theta(theta) {}
    };

    /* HELPER FUNCTIONS */
    /* This function returns the string representation of a cartesian coordinates point */
    const char* to_string(const CartesianPoint& point);
    /* This function returns the string representation of a polar coordinates point */
    const char* to_string(const PolarPoint& point);
    /* This function converts a point from polar_coords to cartesian_coords*/
    CartesianPoint to_cartesian_coordinates(const PolarPoint& polar_point);
    /* This function converts a point from cartesian_coords to polar_coords*/
    PolarPoint to_polar_coordinates(const CartesianPoint& cartesian_point);
    
    /* This function computes the middle point between two points */
    CartesianPoint middle_point(const CartesianPoint& point1, const CartesianPoint& point2);
    /* This function computes the euclidean distance between two points */
    double euclidean_distance(const CartesianPoint& point1, const CartesianPoint& point2);
    /* This function computes the maximum diameter of a distribution of points (i.e. of a pointset) */
    double max_diameter_pointset(const std::vector<CartesianPoint>& points);
    /* This function computes the centroid of a distribution of points (i.e. of a pointset) */
    CartesianPoint centroid_pointset(const std::vector<CartesianPoint>& cluster);
    /* This function computes the distance between the centroid and the farthest point in the distribution of points (i.e. in the pointset) */
    double max_distance_from_centroid(const std::vector<Point::CartesianPoint>& points);


    /* POSITION AND POSE */
    struct Position {
        double x, y, z;

        Position(double x, double y, double z) : x(x), y(y), z(z) {}
        Position(double x, double y) : Position(x, y, 0.0) {}

        /* Overloading of the == operator */
        bool operator==(const Position& other) const;
        /* Overloading of the != operator */
        inline bool operator!=(const Position& other) const { return !(*this == other); }
        /* Overloading of the + operator */
        inline Position operator+(const Position& other) const { return Position(x + other.x, y + other.y, z + other.z); }
        /* Overloading of the - operator */
        inline Position operator-(const Position& other) const { return Position(x - other.x, y - other.y, z - other.z); }
    };

    struct Pose {
        double x, y, z, theta;

        Pose(double x, double y, double z, double theta) : x(x), y(y), z(z), theta(theta) {}
        Pose(double x, double y, double theta) : Pose(x, y, 0.0, theta) {}

        /* Overloading of the == operator */
        bool operator==(const Pose& other) const;
        /* Overloading of the != operator */
        inline bool operator!=(const Pose& other) const { return !(*this == other); }
        /* Overloading of the + operator */
        inline Pose operator+(const Pose& other) const { return Pose(x + other.x, y + other.y, z + other.z, theta + other.theta); }
        /* Overloading of the - operator */
        inline Pose operator-(const Pose& other) const { return Pose(x - other.x, y - other.y, z - other.z, theta - other.theta); }
    };

    /* HELPER FUNCTIONS */
    /* This function returns the string representation of a position point */
    const char* to_string(const Position& position);
    /* This function returns the string representation of a pose point */
    const char* to_string(const Pose& pose);
    /* This function converts Position to Pose */
    Pose to_pose(const Position& position, double theta = 0.0);
    /* This function converts Pose to Position (ignores theta) */
    Position to_position(const Pose& pose);
    /* This function computes the signed angle between two position points in degrees */
    double angle(const Position& p1, const Position& p2);
    /* This function computes the signed angle between two pose points in degrees */
    double angle(const Pose& p1, const Pose& p2);
}

#endif // UTIL_POINT_H