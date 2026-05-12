#ifndef BRAINGRID_H
#define BRAINGRID_H

#include <vector>
#include "Node.h" // Include Node definition

class BrainGrid
{
public:
    int width, height, depth;
    std::vector<std::vector<std::vector<Node *>>> grid;

    BrainGrid(int w, int h, int d);              // Constructor declaration
    void setRisk(int x, int y, int z, int risk); // Method declaration
    std::vector<Node *> getNeighbors(Node *n);   // Method declaration
    ~BrainGrid();                                // Destructor declaration
};

#endif // BRAINGRID_H