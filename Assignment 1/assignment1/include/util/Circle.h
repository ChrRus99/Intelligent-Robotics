#ifndef CIRCLE_H
#define CIRCLE_H

#include "Point.h"
#include <vector>

namespace Geometry {
    struct Circle {
        Point::CartesianPoint center;
        double radius;

        Circle(const Point::CartesianPoint& center, double radius) : center(center), radius(radius) {}
    };

    /* HELPER FUNCTIONS */
    /* This function computes the circumcenter of a triangle formed by three points */
    Point::CartesianPoint circumcenter(const Point::CartesianPoint& p1, const Point::CartesianPoint& p2, const Point::CartesianPoint& p3);
    
    /* This Function checks whether three points are almost collinear */
    bool are_points_collinear(const Point::CartesianPoint& p1, const Point::CartesianPoint& p2, const Point::CartesianPoint& p3, double epsilon = 1e-2);
    
    /* This Function fits a circle through three points */
    Circle fit_circle(const std::vector<Point::CartesianPoint>& points);
}

#endif // CIRCLE_H