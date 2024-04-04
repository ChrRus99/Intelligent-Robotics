#include "HierarchicalClustering.h"
#include <cmath>
#include <iostream>
#include <limits>

using namespace std;
using namespace Point;

vector<vector<CartesianPoint>> HierarchicalClustering::single_linkage(const vector<CartesianPoint>& points, double threshold_distance) {
    vector<vector<CartesianPoint>> clusters;

    // Initialize each point as a cluster
    for (const auto& point : points) {
        clusters.push_back({point});
    }

    while (clusters.size() > 1) {
        double minDistance = numeric_limits<double>::max();
        size_t minCluster1 = 0, minCluster2 = 0;

        // Find the two clusters with the minimum distance
        for (size_t i = 0; i < clusters.size(); ++i) {
            for (size_t j = i + 1; j < clusters.size(); ++j) {
                double distance = calculate_cluster_distance(clusters[i], clusters[j]);
                if (distance < minDistance) {
                    minDistance = distance;
                    minCluster1 = i;
                    minCluster2 = j;
                }
            }
        }

        // Check if the minimum distance is within the specified threshold
        if (minDistance > threshold_distance) {
            break;
        }

        // Merge the two clusters
        clusters[minCluster1].insert(clusters[minCluster1].end(), clusters[minCluster2].begin(), clusters[minCluster2].end());
        clusters.erase(clusters.begin() + minCluster2);
    }

    return clusters;
}

double HierarchicalClustering::calculate_cluster_distance(const vector<CartesianPoint>& cluster1, const vector<CartesianPoint>& cluster2) {
    double minDistance = numeric_limits<double>::max();

    for (const auto& p1 : cluster1) {
        for (const auto& p2 : cluster2) {
            double distance = euclidean_distance(p1, p2);
            if (distance < minDistance) {
                minDistance = distance;
            }
        }
    }

    return minDistance;
}
