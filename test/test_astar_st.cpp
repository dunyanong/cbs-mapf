/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "search_astar_st.hpp" // Include the header for A* search implementation
#include "debug.hpp" // Include debugging utilities
#include <iostream> // For input/output operations
#include <cstdlib> // For std::atoi (convert string to integer)

// Function declaration for testing A* search on a 2D grid
int TestAstarSTGrid2d(int timeout, int grid_size);

int main(int argc, char* argv[]) {
    int timeout = 1; // Default timeout in seconds
    int grid_size = 10; // Default grid size (10x10)

    // Parse command-line arguments
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--timeout" && i + 1 < argc) {
            timeout = std::atoi(argv[++i]); // Set timeout if "--timeout" is provided
        } else if (std::string(argv[i]) == "--grid-size" && i + 1 < argc) {
            grid_size = std::atoi(argv[++i]); // Set grid size if "--grid-size" is provided
        }
    }

    // Print the configuration being used
    std::cout << "Running with timeout: " << timeout << " seconds, grid size: " 
              << grid_size << "x" << grid_size << std::endl;

    // Call the test function with the parsed arguments
    TestAstarSTGrid2d(timeout, grid_size);
    return 0; // Exit the program
};

int TestAstarSTGrid2d(int timeout, int grid_size) {
    // Start of the test function
    std::cout << "####### TestAstarSTGrid2d() Begin #######" << std::endl;

    // Create and start a timer to measure execution time
    raplab::SimpleTimer timer;
    timer.Start();

    // Create a state-space object for the grid
    raplab::StateSpaceST g;

    // Initialize a 2D occupancy grid with all cells set to 0 (free space)
    std::vector<std::vector<double>> occupancy_grid(grid_size, std::vector<double>(grid_size, 0));

    // Set obstacles in the middle row of the grid
    for (int i = 0; i < grid_size; i++) {
        if (i == grid_size / 2) continue; // Skip the middle cell
        occupancy_grid[grid_size / 2][i] = 1; // Mark cell as occupied
    }

    // Set the occupancy grid pointer in the state-space object
    g.SetOccuGridPtr(&occupancy_grid);

    // Create an A* search object
    auto pp = raplab::AstarSTGrid2d();

    // Set the graph pointer for the A* search object
    pp.SetGraphPtr(&g);

    // Set the heuristic weight for the A* algorithm
    pp.SetHeuWeight(1.2);

    // Perform pathfinding from the start node (0) to the goal node (last cell)
    pp.PathFinding(0, grid_size * grid_size - 1, timeout);

    // Print the duration of the pathfinding process
    timer.PrintDuration();

    // End of the test function
    std::cout << "####### TestAstarSTGrid2d() End #######" << std::endl;

    return 1; // Return success
}
