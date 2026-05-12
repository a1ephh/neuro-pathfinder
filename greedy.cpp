#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <unordered_set> // For efficient visited tracking
#include "Node.h"        // Include Node definition
#include "BrainGrid.h"   // Include BrainGrid definition
// Comparator for the Priority Queue
struct CompareHeuristic
{
    bool operator()(Node *a, Node *b)
    {
        return a->h > b->h; // Min-heap: Smallest h (distance) at top
    }
};

std::vector<Node *> findPathGreedy(BrainGrid &brain, Node *start, Node *goal)
{
    // The "Open List" contains nodes we have found but haven't explored yet
    std::priority_queue<Node *, std::vector<Node *>, CompareHeuristic> openList;

    // Track visited nodes to prevent infinite loops
    std::unordered_set<Node *> visited; // Use unordered_set for O(1) average lookup

    start->h = Node::getDistance(start, goal);
    start->parent = nullptr; // Ensure reconstruction terminates at the start node
    visited.insert(start);   // Mark start as visited immediately to prevent parent-pointer cycles
    openList.push(start);

    while (!openList.empty())
    {
        Node *current = openList.top();
        openList.pop();

        // Check if we reached the tumor
        if (current->x == goal->x && current->y == goal->y && current->z == goal->z)
        {
            std::vector<Node *> path;
            size_t maxPathLength = (size_t)brain.width * brain.height * brain.depth;

            while (current != nullptr)
            {
                path.push_back(current);
                current = current->parent;
                if (path.size() > maxPathLength)
                {
                    std::cerr << "Error: Circular path detected in Greedy!" << std::endl;
                    return {};
                }
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Explore neighbors
        for (Node *neighbor : brain.getNeighbors(current))
        {
            // Greedy logic: If not a wall and not already visited
            bool isVisited = visited.count(neighbor) > 0; // O(1) average lookup

            if (neighbor->cost < Node::COST_IMPASSABLE && !isVisited)
            {
                neighbor->parent = current;
                neighbor->h = Node::getDistance(neighbor, goal);
                visited.insert(neighbor); // Add to visited set
                openList.push(neighbor);

                // CRITICAL GREEDY MOVE:
                // In a true greedy search, once we find the "best" next step,
                // we often commit to it immediately to save time.
            }
        }
    }
    return {}; // No path found
}