#ifndef NODE_H
#define NODE_H

#include <limits> // For std::numeric_limits
#include <cmath>  // For std::sqrt and std::pow

struct Node
{
    // Centralized cost definitions for neuro-navigation
    static constexpr double COST_SAFE = 1.0;
    static constexpr double COST_RISK_LIMIT = 50.0;
    static constexpr double COST_IMPASSABLE = 100.0;

    int x, y, z;
    double cost;  // Cost of traversing this node (tissue risk)
    Node *parent; // For path reconstruction

    // For Greedy Best-First Search (heuristic)
    double h;

    // For Dijkstra/A* (cost from start)
    double g;

    Node(int x_coord, int y_coord, int z_coord, double c = COST_SAFE)
        : x(x_coord), y(y_coord), z(z_coord), cost(c), parent(nullptr),
          h(std::numeric_limits<double>::infinity()),
          g(std::numeric_limits<double>::infinity()) {}

    // Helper to calculate Euclidean distance between this node and another
    static double getDistance(Node *a, Node *b)
    {
        return std::sqrt(std::pow(a->x - b->x, 2) + std::pow(a->y - b->y, 2) + std::pow(a->z - b->z, 2));
    }
};

#endif // NODE_H