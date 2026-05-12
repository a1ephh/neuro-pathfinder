#include <vector>
#include <iostream>
#include <algorithm>
#include <limits>
#include "Node.h"
#include "BrainGrid.h"

// Forward declaration of the Dijkstra function from dp.cpp
// This allows D&C to use Dijkstra for local refinement.
std::vector<Node *> findPathDP(BrainGrid &brain, Node *start, Node *goal, bool shouldReset);

struct Block
{
    int x_start, y_start, z_start;
    int size;
    Node *center;
    bool isSafe;
};

/**
 * Divide and Conquer Pathfinding
 * 1. Sub-divides the grid into 5x5 blocks.
 * 2. Pre-processes to identify "Risk-Free" zones.
 * 3. Bridges centers of safe blocks and refines the path locally using Dijkstra.
 */
std::vector<Node *> findPathDC(BrainGrid &brain, Node *start, Node *goal)
{
    const int blockSize = 5;
    std::vector<Block> blocks;
    int numBlocksX = (brain.width + blockSize - 1) / blockSize;
    int numBlocksY = (brain.height + blockSize - 1) / blockSize;

    // 1. Sub-divide & 2. Pre-process
    for (int z = 0; z < brain.depth; z += blockSize)
    {
        for (int y = 0; y < brain.height; y += blockSize)
        {
            for (int x = 0; x < brain.width; x += blockSize)
            {
                bool safe = true;
                for (int bz = z; bz < z + blockSize && bz < brain.depth; ++bz)
                {
                    for (int by = y; by < y + blockSize && by < brain.height; ++by)
                    {
                        for (int bx = x; bx < x + blockSize && bx < brain.width; ++bx)
                        {
                            if (brain.grid[bz][by][bx]->cost >= Node::COST_RISK_LIMIT)
                            {
                                safe = false;
                                break;
                            }
                        }
                        if (!safe)
                            break;
                    }
                    if (!safe)
                        break;
                }

                int actualW = std::min(blockSize, brain.width - x);
                int actualH = std::min(blockSize, brain.height - y);
                int actualD = std::min(blockSize, brain.depth - z);
                Node *centerNode = brain.grid[z + actualD / 2][y + actualH / 2][x + actualW / 2];

                blocks.push_back({x, y, z, blockSize, centerNode, safe});
            }
        }
    }

    // 3. Path-bridge: Identify high-level waypoints
    auto getBlockIdx = [&](Node *n)
    {
        return (n->z / blockSize) * (numBlocksX * numBlocksY) + (n->y / blockSize) * numBlocksX + (n->x / blockSize);
    };

    int startIdx = getBlockIdx(start);
    int goalIdx = getBlockIdx(goal);

    // Optimized Bridging: Create waypoints along the line to "Divide" the path
    std::vector<Node *> waypoints;
    waypoints.push_back(start);

    if (startIdx != goalIdx)
    {
        // Simple linear interpolation of block centers for demonstration
        int sx = start->x / blockSize, sy = start->y / blockSize, sz = start->z / blockSize;
        int gx = goal->x / blockSize, gy = goal->y / blockSize, gz = goal->z / blockSize;

        int steps = std::max({abs(gx - sx), abs(gy - sy), abs(gz - sz)});
        for (int i = 1; i < steps; ++i)
        {
            int tx = sx + (gx - sx) * i / steps;
            int ty = sy + (gy - sy) * i / steps;
            int tz = sz + (gz - sz) * i / steps;
            int idx = tz * (numBlocksX * numBlocksY) + ty * numBlocksX + tx;
            if (idx >= 0 && idx < blocks.size() && blocks[idx].isSafe)
            {
                waypoints.push_back(blocks[idx].center);
            }
        }
    }
    waypoints.push_back(goal);

    // Refine: Use Dijkstra to stitch the waypoints together
    std::vector<Node *> fullPath;
    // Reset the grid once for the whole D&C operation
    for (auto &plane : brain.grid)
    {
        for (auto &row : plane)
        {
            for (Node *node : row)
            {
                node->g = std::numeric_limits<double>::infinity();
                node->parent = nullptr;
            }
        }
    }

    for (size_t i = 0; i < waypoints.size() - 1; ++i)
    {
        if (waypoints[i] == waypoints[i + 1])
            continue;

        std::vector<Node *> segment = findPathDP(brain, waypoints[i], waypoints[i + 1], false);
        if (segment.empty())
            return {}; // Failed to bridge safe zones

        // Combine segments and remove duplicate junction nodes
        if (!fullPath.empty())
            fullPath.pop_back();
        fullPath.insert(fullPath.end(), segment.begin(), segment.end());
    }

    return fullPath;
}