#include "util/Point.h"
#include <cmath>

using namespace std;
using namespace Point;

namespace Point {
    CartesianPoint polar2cartesian_coords(const PolarPoint& polar_point) {
        double x = polar_point.rho * cos(polar_point.theta);
        double y = polar_point.rho * sin(polar_point.theta);

        return CartesianPoint(x, y);
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
        size_t numPoints = points.size();
        if (numPoints > 0) {
            centroid.x /= numPoints;
            centroid.y /= numPoints;
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
}