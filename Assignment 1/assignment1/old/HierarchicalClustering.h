#ifndef HIERARCHICAL_CLUSTERING_H
#define HIERARCHICAL_CLUSTERING_H

#include <vector>
#include "Point.h"

class HierarchicalClustering {
public:
    std::vector<std::vector<Point::CartesianPoint>> single_linkage(const std::vector<Point::CartesianPoint>& points, double threshold_distance);

private:
    double calculate_cluster_distance(const std::vector<Point::CartesianPoint>& cluster1, const std::vector<Point::CartesianPoint>& cluster2);
};

#endif // HIERARCHICAL_CLUSTERING_H
