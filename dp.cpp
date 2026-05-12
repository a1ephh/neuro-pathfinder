#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <iostream>    // Added for std::cerr
#include "Node.h"      // Include Node definition
#include "BrainGrid.h" // Include BrainGrid definition

// In DP/Dijkstra, we only care about the accumulated g-score (cost from start)
struct CompareDP
{
    // Store pair of <cost, Node*> to handle stale entries in the priority queue
    typedef std::pair<double, Node *> PQEntry;
    bool operator()(const PQEntry &a, const PQEntry &b)
    {
        return a.first > b.first; // Min-heap based on cost
    }
};

std::vector<Node *> findPathDP(BrainGrid &brain, Node *start, Node *goal, bool shouldReset = true)
{
    // Priority queue to always expand the "cheapest" known subproblem
    std::priority_queue<CompareDP::PQEntry, std::vector<CompareDP::PQEntry>, CompareDP> pq;

    // Only reset the whole grid if requested (D&C uses this to avoid O(V) per segment)
    if (shouldReset)
        for (auto &plane : brain.grid)
        {
            for (auto &row : plane)
            {
                for (auto &node : row)
                {
                    node->g = std::numeric_limits<double>::infinity();
                    node->parent = nullptr;
                }
            }
        }

    start->g = 0;
    start->parent = nullptr; // Ensure reconstruction terminates at the start node
    pq.push({0.0, start});

    while (!pq.empty())
    {
        double currentG = pq.top().first;
        Node *current = pq.top().second;
        pq.pop();

        // Skip if we already found a better path to this node
        if (currentG > current->g)
            continue;

        // Optimization: if we reached the goal, we can stop
        if (current == goal)
            break;

        for (Node *neighbor : brain.getNeighbors(current))
        {
            double weight = neighbor->cost;
            if (weight >= Node::COST_IMPASSABLE)
                continue; // Skip impassable zones

            // World-class improvement: Cost = Distance * Tissue Risk
            double moveDist = Node::getDistance(current, neighbor);
            double newDist = current->g + (moveDist * weight);

            // Relaxation Step: If we found a cheaper way to reach 'neighbor'
            if (newDist < neighbor->g)
            {
                neighbor->g = newDist;
                neighbor->parent = current;
                pq.push({newDist, neighbor});
            }
        }
    }

    // Reconstruct the path
    std::vector<Node *> path;

    // If the goal was never reached, return an empty vector
    if (goal->g == std::numeric_limits<double>::infinity())
    {
        return path;
    }

    // Safety: prevent infinite loop if a cycle somehow formed
    size_t maxPathLength = (size_t)brain.width * brain.height * brain.depth;
    Node *temp = goal;
    while (temp != nullptr)
    {
        path.push_back(temp);
        temp = temp->parent;

        if (path.size() > maxPathLength)
        {
            std::cerr << "Error: Circular path detected in Dijkstra!" << std::endl;
            return {};
        }
    }
    std::reverse(path.begin(), path.end());
    return path;
}