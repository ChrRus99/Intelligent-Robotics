#include "util/Circle.h"
#include <cmath>
#include <stdexcept>

using namespace std;
using namespace Point;

namespace Geometry {
    CartesianPoint circumcenter(const CartesianPoint& p1, const CartesianPoint& p2, const CartesianPoint& p3) {
        double x1 = p1.x, y1 = p1.y;
        double x2 = p2.x, y2 = p2.y;
        double x3 = p3.x, y3 = p3.y;

        double D = 2.0 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
        
        double Ox = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) + (x3 * x3 + y3 * y3) * (y1 - y2)) / D;
        double Oy = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) + (x3 * x3 + y3 * y3) * (x2 - x1)) / D;

        return CartesianPoint(Ox, Oy);
    }

    bool are_points_collinear(const CartesianPoint& p1, const CartesianPoint& p2, const CartesianPoint& p3, double epsilon) {
        double det = (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y);
        return abs(det) < epsilon;
    }

    Circle fit_circle(const vector<CartesianPoint>& points) {
        // Check whether the vector contains enough points to form a circle
        if (points.size() < 3) {
            throw invalid_argument("At least three points are required to fit a circle.");
        }

        // Choose the first, middle, and last points in the vector
        const CartesianPoint& p1 = points.front();
        const CartesianPoint& p2 = points[points.size() / 2];
        const CartesianPoint& p3 = points.back();

        // Check whether the points are almost collinear
        if (are_points_collinear(p1, p2, p3)) {
            throw invalid_argument("Points are almost collinear: fitting a circle may not be meaningful.");
        }

        // Find the circumference center of the triangle formed by these three points
        CartesianPoint center = circumcenter(p1, p2, p3);

        // Compute the radius using the distance from the circumcenter to any of the points
        double radius = sqrt((center.x - p1.x) * (center.x - p1.x) + (center.y - p1.y) * (center.y - p1.y));

        return Circle(center, radius);
    }
}