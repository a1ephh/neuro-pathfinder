#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>

#include "Node.h"
#include "BrainGrid.h"
#include "brainGrid.cpp"
#include "greedy.cpp"
#include "dp.cpp"
#include "divide_conquer.cpp"

void printPath(const std::vector<Node *> &path, const std::string &algorithmName)
{
    if (path.empty())
    {
        std::cout << algorithmName << ": No path found." << std::endl;
        return;
    }

    std::cout << algorithmName << " Path: ";
    for (size_t i = 0; i < path.size(); ++i)
    {
        std::cout << "(" << path[i]->x << "," << path[i]->y << "," << path[i]->z << ")";
        if (i < path.size() - 1)
        {
            std::cout << " -> ";
        }
    }
    std::cout << std::endl;

    double totalCost = 0.0;
    if (!path.empty())
    {
        //  Path Cost = Sum(Step Distance * Node Risk)
        for (size_t i = 1; i < path.size(); ++i)
        {
            double stepDist = Node::getDistance(path[i - 1], path[i]);
            totalCost += stepDist * path[i]->cost;
        }
        std::cout << "Total path length (nodes): " << path.size() << std::endl;
        std::cout << "Total path cost (sum of node costs): " << totalCost << std::endl;
    }
    std::cout << std::endl;
}

// Function to reset node states for a new search
void resetGridNodes(BrainGrid &brain)
{
    for (auto &plane : brain.grid)
    {
        for (auto &row : plane)
        {
            for (Node *node : row)
            {
                node->parent = nullptr;
                node->g = std::numeric_limits<double>::infinity();
                node->h = std::numeric_limits<double>::infinity();
            }
        }
    }
}

/**
 * Creates sample test case documents to demonstrate the file-reading capability.
 */
void createTestFiles()
{
    // Test Case 1: The "Vascular Barrier" (Optimality Test)
    std::ofstream f1("test_vascular.txt");
    if (!f1.is_open())
    {
        std::cerr << "Error: Failed to create test_vascular.txt" << std::endl;
        return;
    }
    f1 << "10 10 10\n0 0 0\n9 9 9\n"; // 10x10x10, Start (0,0,0), Goal (9,9,9)
    // Place Vessel Barrier at Z=5, X and Y from 4 to 6
    for (int y = 4; y <= 6; ++y)
    {
        for (int x = 4; x <= 6; ++x)
        {
            f1 << x << " " << y << " 5 50\n";
        }
    }
    f1.close();

    // Test Case 2: The "Divide & Conquer" Block Test
    std::ofstream f2("test_dc_block.txt");
    if (!f2.is_open())
    {
        std::cerr << "Error: Failed to create test_dc_block.txt" << std::endl;
        return;
    }
    f2 << "10 10 10\n0 0 0\n5 5 0\n"; // Start (0,0,0), Goal (5,5,0)
    f2.close();

    // Test Case 3: The "Narrow Corridor" (Constraint Navigation)
    std::ofstream f3("test_corridor.txt");
    f3 << "10 10 10\n0 0 0\n9 9 9\n";
    // Create a wall at X=5 with a single hole at (5,5,5)
    for (int z = 0; z < 10; ++z)
    {
        for (int y = 0; y < 10; ++y)
        {
            if (y == 5 && z == 5)
                continue; // The hole
            f3 << "5 " << y << " " << z << " 100\n";
        }
    }
    f3.close();

    // Test Case 4: The "Risk Shell" (Minimum Risk Path)
    std::ofstream f4("test_shell.txt");
    f4 << "10 10 10\n0 0 0\n5 5 5\n";
    // Create a 3x3x3 shell of risk around the goal (5,5,5)
    for (int z = 4; z <= 6; ++z)
    {
        for (int y = 4; y <= 6; ++y)
        {
            for (int x = 4; x <= 6; ++x)
            {
                if (x == 5 && y == 5 && z == 5)
                    continue; // The goal itself
                // Leave a "weak point" at (5,4,5) with lower risk
                if (x == 5 && y == 4 && z == 5)
                    f4 << x << " " << y << " " << z << " 10\n";
                else
                    f4 << x << " " << y << " " << z << " 50\n";
            }
        }
    }
    f4.close();

    std::cout << "[System] Test case documents created (Vascular, DC Block, Corridor, Shell).\n";
}

/**
 * Reads a scenario from a file and executes pathfinding algorithms.
 */
void runSimulationFromFile(const std::string &filename)
{
    std::ifstream infile(filename);
    if (!infile.is_open())
    {
        std::cerr << "Error: Could not open " << filename << std::endl;
        return;
    }

    int w, h, d, sx, sy, sz, gx, gy, gz;
    if (!(infile >> w >> h >> d >> sx >> sy >> sz >> gx >> gy >> gz))
    {
        std::cerr << "Error: Invalid file header in " << filename << std::endl;
        return;
    }

    BrainGrid brain(w, h, d);
    int rx, ry, rz, rval;
    while (infile >> rx >> ry >> rz >> rval)
    {
        brain.setRisk(rx, ry, rz, rval);
    }
    infile.close();

    if (sx < 0 || sx >= w || sy < 0 || sy >= h || sz < 0 || sz >= d ||
        gx < 0 || gx >= w || gy < 0 || gy >= h || gz < 0 || gz >= d)
    {
        std::cerr << "Error: Start/Goal coordinates out of bounds in " << filename << std::endl;
        return;
    }

    Node *startNode = brain.grid[sz][sy][sx];
    Node *goalNode = brain.grid[gz][gy][gx];

    std::cout << "\n--- Running Simulation: " << filename << " ---" << std::endl;
    std::cout << "Grid: " << w << "x" << h << "x" << d << " | Start: (" << sx << "," << sy << "," << sz << ") | Goal: (" << gx << "," << gy << "," << gz << ")" << std::endl;

    resetGridNodes(brain);
    std::vector<Node *> greedyPath = findPathGreedy(brain, startNode, goalNode);
    printPath(greedyPath, "Greedy Search");

    resetGridNodes(brain);
    std::vector<Node *> dpPath = findPathDP(brain, startNode, goalNode);
    printPath(dpPath, "Dijkstra's Algorithm");

    // --- Divide & Conquer ---
    resetGridNodes(brain);
    std::vector<Node *> dcPath = findPathDC(brain, startNode, goalNode);
    printPath(dcPath, "Divide & Conquer");
    std::cout << "------------------------------------------------" << std::endl;
}

/**
 * Measures runtime for each algorithm as the grid size increases.
 */
void runPerformanceTest()
{
    std::cout << "\n--- Performance Benchmark: Runtime vs. Input Size ---" << std::endl;
    std::cout << std::left << std::setw(15) << "Grid Size (N^3)"
              << std::setw(15) << "Greedy (ms)"
              << std::setw(15) << "Dijkstra (ms)"
              << std::setw(15) << "D&C (ms)" << std::endl;
    std::cout << std::string(60, '-') << std::endl;

    // Scaling up to 80^3 (512,000 nodes) to show D&C efficiency
    for (int size : {20, 40, 60, 80})
    {
        BrainGrid brain(size, size, size);
        Node *start = brain.grid[0][0][0];
        Node *goal = brain.grid[size - 1][size - 1][size - 1];

        auto measure = [&](auto func)
        {
            resetGridNodes(brain);
            auto t1 = std::chrono::high_resolution_clock::now();
            func();
            auto t2 = std::chrono::high_resolution_clock::now();
            return std::chrono::duration<double, std::milli>(t2 - t1).count();
        };

        double timeGreedy = measure([&]()
                                    { findPathGreedy(brain, start, goal); });
        double timeDP = measure([&]()
                                { findPathDP(brain, start, goal); });
        double timeDC = measure([&]()
                                { findPathDC(brain, start, goal); });

        std::cout << std::left << std::setw(15) << (std::to_string(size) + "^3")
                  << std::setw(15) << std::fixed << std::setprecision(3) << timeGreedy
                  << std::setw(15) << timeDP
                  << std::setw(15) << timeDC << std::endl;
    }
    std::cout << "----------------------------------------------------" << std::endl;
}

int main()
{
    std::cout << "Neuro-Pathfinder Research Tool" << std::endl;

    // Ask user what to run to avoid long wait times on large benchmarks
    std::cout << "1. Run Sample Scenarios (from files)\n";
    std::cout << "2. Run Performance Benchmark (Runtime vs Size)\n";
    std::cout << "Enter choice: ";
    int choice;
    if (!(std::cin >> choice))
        choice = 1;

    if (choice == 1)
    {
        createTestFiles();
        runSimulationFromFile("test_vascular.txt");
        runSimulationFromFile("test_dc_block.txt");
        runSimulationFromFile("test_corridor.txt");
        runSimulationFromFile("test_shell.txt");
    }
    else
    {
        runPerformanceTest();
    }

    std::cout << "\n[System] Execution complete." << std::endl;
    return 0;
}