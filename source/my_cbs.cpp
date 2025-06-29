#include "my_cbs.hpp"
#include <iostream>

namespace raplab {
    CBS::CBS(PlannerGraph* input_graph, int num_agents) {
        this->input_graph = input_graph;
        this->num_agents = num_agents;
        std::cout << "[DEBUG] CBS initialized with " << num_agents << " agents." << std::endl;
    }

    std::vector<std::vector<long>> CBS::Solve(
        const std::vector<long>& starts,
        const std::vector<long>& goals
    ) {
        std::cout << "[DEBUG] Starting CBS Solve..." << std::endl;

        // Priority queue for CBS nodes
        std::priority_queue<CBSNode> priority_queue_open_list;

        // Initialize CBS node
        CBSNode root;
        root.paths.resize(num_agents);
        root.cost = 0.0;

        // Perform low-level search for each agent
        for (int i = 0; i < num_agents; i++) {
            std::cout << "[DEBUG] Performing low-level search for agent " << i 
                      << " from " << starts[i] << " to " << goals[i] << "." << std::endl;
            root.paths[i] = LowLevelSearch(i, root.constraints, starts[i], goals[i]);
            root.cost += root.paths[i].size();
            std::cout << "[DEBUG] Agent " << i << " path cost: " << root.paths[i].size() << std::endl;
        }

        priority_queue_open_list.push(root);
        std::cout << "[DEBUG] Initial CBS node pushed to priority queue with cost: " << root.cost << std::endl;

        while (!priority_queue_open_list.empty()) {
            CBSNode current_node = priority_queue_open_list.top();
            priority_queue_open_list.pop();
            std::cout << "[DEBUG] Expanding CBS node with cost: " << current_node.cost << std::endl;

            int agent1, agent2;
            long conflict_vertex, conflict_time;
            if (!DetectConflict(current_node.paths, agent1, agent2, conflict_vertex, conflict_time)) {
                std::cout << "[DEBUG] No conflicts found. Solution is valid." << std::endl;
                return current_node.paths;
            }

            std::cout << "[DEBUG] Conflict detected between agent " << agent1 
                      << " and agent " << agent2 
                      << " at vertex " << conflict_vertex 
                      << " at time " << conflict_time << "." << std::endl;

            for (int i = 0; i < 2; ++i) {
                // Create a new child node
                CBSNode child = current_node;

                // Add constraints for the conflicting agents
                Constraint new_constraint = {
                    i == 0 ? agent1 : agent2,
                    conflict_vertex,
                    conflict_time
                };
                child.constraints.push_back(new_constraint);
                std::cout << "[DEBUG] Adding constraint for agent " 
                          << (i == 0 ? agent1 : agent2) 
                          << ": vertex " << conflict_vertex 
                          << ", time " << conflict_time << "." << std::endl;

                // Re-plan for the agent involved in the conflict
                child.paths[i == 0 ? agent1 : agent2] = LowLevelSearch(
                    i == 0 ? agent1 : agent2,
                    child.constraints,
                    starts[i == 0 ? agent1 : agent2],
                    goals[i == 0 ? agent1 : agent2]
                );

                child.cost = 0.0;
                for (const auto& path : child.paths) {
                    child.cost += path.size();
                }
                std::cout << "[DEBUG] Child node created with cost: " << child.cost << std::endl;

                priority_queue_open_list.push(child);
            }
        }

        std::cout << "[ERROR] No solution found. Returning empty paths." << std::endl;
        return {}; // Return empty paths if no solution is found
    }

    std::vector<long> CBS::LowLevelSearch(
        int agent,
        const std::vector<Constraint>& constraints,
        long start,
        long goal
    ) {
        std::cout << "[DEBUG] Starting low-level search for agent " << agent 
                  << " from " << start << " to " << goal << "." << std::endl;

        AstarSTGrid2d planner;
        planner.SetGraphPtr(input_graph);

        for (const auto& constraint : constraints) {
            if (constraint.agent == agent) {
                planner.AddNodeCstr(constraint.vertex, constraint.time);
                std::cout << "[DEBUG] Adding constraint for agent " << agent 
                          << ": vertex " << constraint.vertex 
                          << ", time " << constraint.time << "." << std::endl;
            }
        }

        auto path = planner.PathFinding(start, goal, std::numeric_limits<double>::infinity());
        std::cout << "[DEBUG] Low-level search completed for agent " << agent 
                  << ". Path length: " << path.size() << std::endl;
        return path;
    }

    bool CBS::DetectConflict(
        const std::vector<std::vector<long>>& paths,
        int& agent1,
        int& agent2,
        long& conflict_vertex,
        long& conflict_time
    ) {
        std::cout << "[DEBUG] Checking for conflicts..." << std::endl;

        for (int i = 0; i < paths.size(); ++i) {
            for (int j = i + 1; j < paths.size(); ++j) {
                for (int t = 0; t < std::min(paths[i].size(), paths[j].size()); ++t) {
                    if (paths[i][t] == paths[j][t]) {
                        agent1 = i;
                        agent2 = j;
                        conflict_vertex = paths[i][t];
                        conflict_time = t;
                        std::cout << "[DEBUG] Conflict detected: agent " << agent1 
                                  << " and agent " << agent2 
                                  << " at vertex " << conflict_vertex 
                                  << " at time " << conflict_time << "." << std::endl;
                        return true;
                    }
                }
            }
        }

        std::cout << "[DEBUG] No conflicts detected." << std::endl;
        return false;
    }
}