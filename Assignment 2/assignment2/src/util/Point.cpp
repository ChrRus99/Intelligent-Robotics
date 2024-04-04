#include "util/Point.h"
#include <cmath>
#include <sstream>
#include <cstring>
#include <iomanip>

using namespace std;

namespace Point {
    /* CARTESIAN AND POLAR POINTS */
    const char* to_string(const CartesianPoint& point) {
        ostringstream oss;
        oss << fixed << setprecision(2);
        oss << "{x: " << point.x << ", y: " << point.y << "}";
        
        // Allocate memory for the C-style string and copy the content
        char* result = strdup(oss.str().c_str());

        return result;
    }

    const char* to_string(const PolarPoint& point) {
        ostringstream oss;
        oss << fixed << setprecision(2);
        oss << "{rho: " << point.rho << ", theta: " << point.theta << "}";
        
        // Allocate memory for the C-style string and copy the content
        char* result = strdup(oss.str().c_str());

        return result;
    }

    CartesianPoint to_cartesian_coordinates(const PolarPoint& polar_point) {
        double x = polar_point.rho * cos(polar_point.theta);
        double y = polar_point.rho * sin(polar_point.theta);
        return CartesianPoint(x, y);
    }

    PolarPoint to_polar_coordinates(const CartesianPoint& cartesian_point) {
        double rho = sqrt(cartesian_point.x * cartesian_point.x + cartesian_point.y * cartesian_point.y);
        double theta = atan2(cartesian_point.y, cartesian_point.x); // in radiants
        return PolarPoint(rho, theta);
    }

    CartesianPoint middle_point(const CartesianPoint& point1, const CartesianPoint& point2) {
        double x = (point1.x + point2.x) / 2.0;
        double y = (point1.y + point2.y) / 2.0;

        return CartesianPoint(x, y);
    }

    double euclidean_distance(const CartesianPoint& point1, const CartesianPoint& point2) {
        double dx = point1.x - point2.x;
        double dy = point1.y - point2.y;

        return sqrt(dx * dx + dy * dy);
    }

    double max_diameter_pointset(const vector<CartesianPoint>& points) {
        double max_diameter = 0.0;

        // Compute the maximum diameter as the maximum distance between two points in the pointset
        for (size_t i = 0; i < points.size(); i++) {
            for (size_t j = i + 1; j < points.size(); j++) {
                double diameter = euclidean_distance(points[i], points[j]);
                if (diameter > max_diameter) {
                    max_diameter = diameter;
                }
            }
        }
        
        return max_diameter;
    }

    CartesianPoint centroid_pointset(const vector<CartesianPoint>& points) {
        CartesianPoint centroid = {0.0, 0.0};

        // Sum up the coordinates of all points in the pointset
        for (const auto& point : points) {
            centroid.x += point.x;
            centroid.y += point.y;
        }

        // Divide by the number of points to get the average coordinates
        size_t num_points = points.size();
        if (num_points > 0) {
            centroid.x /= num_points;
            centroid.y /= num_points;
        }

        return centroid;
    }

    double max_distance_from_centroid(const vector<CartesianPoint>& points) {
        // Calculate the centroid of the pointset
        CartesianPoint centroid = centroid_pointset(points);

        double max_distance = 0.0;

        // Calculate the distance of each point from the centroid and find the maximum
        for (const auto& point : points) {
            double distance = euclidean_distance(centroid, point);
            if (distance > max_distance) {
                max_distance = distance;
            }
        }

        return max_distance;
    }


    /* POSITION AND POSE */
    bool Position::operator==(const Position& other) const {
        return ((abs(x - other.x) < 1e-6) || (isinf(x) && isinf(other.x))) &&
               ((abs(y - other.y) < 1e-6) || (isinf(y) && isinf(other.y))) &&
               ((abs(z - other.z) < 1e-6) || (isinf(z) && isinf(other.z)));
    }

    bool Pose::operator==(const Pose& other) const {
        return ((abs(x - other.x) < 1e-6) || (isinf(x) && isinf(other.x))) &&
               ((abs(y - other.y) < 1e-6) || (isinf(y) && isinf(other.y))) &&
               ((abs(z - other.z) < 1e-6) || (isinf(z) && isinf(other.z))) &&
               ((abs(theta - other.theta) < 1e-6) || (isinf(theta) && isinf(other.theta)));
    }
    
    const char* to_string(const Position& position) {
        ostringstream oss;
        oss << fixed << setprecision(2);
        oss << "{x: " << position.x << ", y: " << position.y << ", z: " << position.z << "}";
        
        // Allocate memory for the C-style string and copy the content
        char* result = strdup(oss.str().c_str());

        return result;
    }

    const char* to_string(const Pose& pose) {
        ostringstream oss;
        oss << fixed << setprecision(2);
        oss << "{x: " << pose.x << ", y: " << pose.y << ", z: " << pose.z << ", theta: " << pose.theta << "}";
        
        // Allocate memory for the C-style string and copy the content
        char* result = strdup(oss.str().c_str());

        return result;
    }

    Pose to_pose(const Position& position, double theta) {
        return Pose(position.x, position.y, position.z, theta);
    }
    
    Position to_position(const Pose& pose) {
        return Position(pose.x, pose.y, pose.z);
    }

    double angle(const Position& p1, const Position& p2) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;

        // Avoid division by zero
        if (dx == 0.0 && dy == 0.0) {
            return 0.0;
        }

        double angle = atan2(dy, dx) * (180.0 / M_PI);

        // Ensure the angle is in the range [-180, 180)
        if (angle < -180.0) {
            angle += 360.0;
        } else if (angle >= 180.0) {
            angle -= 360.0;
        }

        return angle; // in degrees
    }

    double angle(const Pose& p1, const Pose& p2) {
        Position position1(p1.x, p1.y, p1.z);
        Position position2(p2.x, p2.y, p2.z);

        return angle(position1, position2); // in degrees
    }
}