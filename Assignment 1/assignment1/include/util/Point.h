#ifndef UTIL_POINT_H
#define UTIL_POINT_H

#include <vector>

namespace Point {
    struct CartesianPoint {
        double x, y;

        CartesianPoint(double x, double y) : x(x), y(y) {}
    };

    struct PolarPoint {
        double rho, theta;

        PolarPoint(double rho, double theta) : rho(rho), theta(theta) {}
    };

    /* HELPER FUNCTIONS */
    /* This function converts a point from polar_coords to cartesian_coords*/
    CartesianPoint polar2cartesian_coords(const PolarPoint& polar_point);
    
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
}

#endif // UTIL_POINT_H