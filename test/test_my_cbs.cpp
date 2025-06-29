#include "my_cbs.hpp"
#include "graph.hpp"
#include "search_astar_st.hpp"
#include <iostream>

int main() {
    // Create a StateSpaceST object
    raplab::StateSpaceST state_space;

    // Set the occupancy grid for the state space
    state_space.SetOccuGridPtr(
        new std::vector<std::vector<double>>(10, std::vector<double>(10, 0.0))
    );

    // 2 agents
    int no_of_agents = 2;

    // Pass the StateSpaceST object to CBS
    raplab::CBS cbs(&state_space, no_of_agents);
    
    // Agent 0: Start at 0 → Goal at 99
    std::vector<long> starts = {0, 99};

    // Agent 1: Start at 99 → Goal at 0
    std::vector<long> goals = {99, 0};

    std::cout << "[DEBUG] Starting CBS Solve..." << std::endl;
    // Solve the multi-agent pathfinding problem
    auto paths = cbs.Solve(starts, goals);
    std::cout << "[DEBUG] CBS Solve completed." << std::endl;

    // Print the paths for each agent
    for (size_t i = 0; i < paths.size(); ++i) {
        std::cout << "Agent " << i << " path: ";
        for (const auto& vertex : paths[i]) {
            std::cout << vertex << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}