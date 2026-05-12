#include <vector>
#include "Node.h"
#include "BrainGrid.h" // Include the header for BrainGrid

BrainGrid::BrainGrid(int w, int h, int d) : width(w), height(h), depth(d)
{
    for (int k = 0; k < depth; ++k)
    {
        std::vector<std::vector<Node *>> plane;
        for (int i = 0; i < height; ++i)
        {
            std::vector<Node *> row;
            for (int j = 0; j < width; ++j)
            {
                row.push_back(new Node(j, i, k, 1.0));
            }
            plane.push_back(row);
        }
        grid.push_back(plane);
    }
}

void BrainGrid::setRisk(int x, int y, int z, int risk)
{
    if (x >= 0 && x < width && y >= 0 && y < height && z >= 0 && z < depth)
        grid[z][y][x]->cost = risk;
}

std::vector<Node *> BrainGrid::getNeighbors(Node *n)
{
    std::vector<Node *> neighbors;
    for (int dz = -1; dz <= 1; ++dz)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dx = -1; dx <= 1; ++dx)
            {
                if (dx == 0 && dy == 0 && dz == 0)
                    continue;
                int nx = n->x + dx;
                int ny = n->y + dy;
                int nz = n->z + dz;
                if (nx >= 0 && nx < width && ny >= 0 && ny < height && nz >= 0 && nz < depth)
                {
                    neighbors.push_back(grid[nz][ny][nx]);
                }
            }
        }
    }
    return neighbors;
}

BrainGrid::~BrainGrid()
{
    for (auto &plane : grid)
    {
        for (auto &row : plane)
        {
            for (auto n : row)
                delete n;
        }
    }
}